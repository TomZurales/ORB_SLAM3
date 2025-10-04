#include "DatabaseManager.h"

#include <mutex>
#include <fstream>

std::unique_ptr<DatabaseManager> DatabaseManager::_instance = nullptr;

DatabaseManager &DatabaseManager::Instance()
{
    if (!_instance)
        throw std::runtime_error("DatabaseManager not initialized. Call DatabaseManager::Init(path) first.");
    return *_instance;
}

void DatabaseManager::Init(const std::string &test_name)
{
    if (!_instance)
        _instance.reset(new DatabaseManager(test_name));
}

DatabaseManager::DatabaseManager(const std::string &test_name)
{

    try
    {
        _common_db = std::make_unique<SQLite::Database>("common.db", SQLite::OPEN_READWRITE);
        _common_db->exec("PRAGMA busy_timeout = 5000;");
    }
    catch (const SQLite::Exception &e)
    {
        std::cerr << "Error opening common database: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string dbPath = test_name + ".db";
    _test_db = std::make_unique<SQLite::Database>(dbPath, SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
    _test_db->exec("PRAGMA busy_timeout = 5000;");

    if (_test_db->tableExists("Params"))
    {
        std::cout << "Loading pre-existing test: " << dbPath << std::endl;
        _clearTestData();
    }
    else
    {
        _initTestDB();
        _interactivePopulateTestDB();
    }

    stop_worker = false;
    worker_thread = std::thread(&DatabaseManager::workerLoop, this);
}
DatabaseManager::~DatabaseManager()
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        stop_worker = true;
    }
    queue_cv.notify_one();
    if (worker_thread.joinable())
        worker_thread.join();
}
void DatabaseManager::enqueue(Task task)
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        task_queue.push(std::move(task));
    }
    queue_cv.notify_one();
}

bool DatabaseManager::workerIsDone() const
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    return task_queue.empty();
}

void DatabaseManager::workerLoop()
{
    while (true)
    {
        Task task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cv.wait(lock, [this]
                          { return stop_worker || !task_queue.empty(); });
            if (stop_worker && task_queue.empty())
                break;
            task = std::move(task_queue.front());
            task_queue.pop();
        }
        if (task)
        {
            std::lock_guard<std::mutex> dblock(db_access_mutex);
            task();
        }
    }
}

void DatabaseManager::_clearTestData()
{
    _test_db->exec("DELETE FROM VBEE;DELETE FROM FramePoses;DELETE FROM KeyframePoses;DELETE FROM RANSACStats");
}

void DatabaseManager::_interactivePopulateTestDB()
{
    std::cout << "Test Setup: " << std::endl;

    std::vector<Trajectory> trajectories = getAllTrajectories();
    std::vector<int> selected;

    while (true)
    {
        // Print list with selection marks
        for (const auto &t : trajectories)
        {
            std::cout << t.id << ": " << t.name;
            if (std::find(selected.begin(), selected.end(), t.id) != selected.end())
                std::cout << " X";
            std::cout << std::endl;
        }

        std::cout << "Type dataset id to toggle selection, or 'done' to finish: ";
        std::string line;
        if (!std::getline(std::cin, line))
            break; // EOF
        if (line == "done")
            break;

        // try parse int
        try
        {
            int id = std::stoi(line);
            // check id exists
            bool found = false;
            for (const auto &t : trajectories)
            {
                if (t.id == id)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                std::cout << "No dataset with id " << id << std::endl;
                continue;
            }
            auto it = std::find(selected.begin(), selected.end(), id);
            if (it != selected.end())
                selected.erase(it);
            else
                selected.push_back(id);
        }
        catch (...)
        {
            std::cout << "Invalid input" << std::endl;
        }
    }

    for (const auto &id : selected)
    {
        addTrajectoryToTest(id);
    }

    // Prompt as to whether to use VBEE for this test. If not, end. Otherwise prompt for all the parameters in the Params table.
    std::string line;
    std::cout << "Use VBEE for this test? (y/n) [y]: ";
    if (!std::getline(std::cin, line))
        return;
    bool use_vbee = true;
    if (!line.empty() && (line == "n" || line == "N" || line == "no" || line == "NO"))
    {
        use_vbee = false;
        std::cout << "VBEE disabled for this test. A Params row will still be added with defaults and use_vbee=0.\n";
    }

    std::string line2;
    std::cout << "Weight RANSAC for this test? (y/n) [y]: ";
    if (!std::getline(std::cin, line))
        return;
    bool weight_ransac = true;
    if (!line2.empty() && (line2 == "n" || line2 == "N" || line2 == "no" || line2 == "NO"))
    {
        weight_ransac = false;
        std::cout << "RANSAC weighting disabled for this test.\n";
    }

    // Defaults
    const int def_k = 150;
    const int def_n = 15;
    const double def_a_th = 0.52;
    const double def_f_th = 0.1;
    const double def_init_p_e = 0.9;
    const double def_damp_coeff = 0.3;
    const double def_init_obs = 0.5;
    const double def_obs_damp_coeff = 0.4;

    std::string name;
    std::cout << "Test name [vbee_test]: ";
    if (!std::getline(std::cin, name))
        return;
    if (name.empty())
        name = "vbee_test";

    auto read_int = [&](const std::string &prompt, int def) -> int
    {
        std::cout << prompt << " [" << def << "]: ";
        std::string s;
        std::getline(std::cin, s);
        if (s.empty())
            return def;
        try
        {
            return std::stoi(s);
        }
        catch (...)
        {
            std::cout << "Invalid int, using default.\n";
            return def;
        }
    };
    auto read_double = [&](const std::string &prompt, double def) -> double
    {
        std::cout << prompt << " [" << def << "]: ";
        std::string s;
        std::getline(std::cin, s);
        if (s.empty())
            return def;
        try
        {
            return std::stod(s);
        }
        catch (...)
        {
            std::cout << "Invalid number, using default.\n";
            return def;
        }
    };

    int k = def_k;
    int n = def_n;
    double a_th = def_a_th;
    double f_th = def_f_th;
    double init_p_e = def_init_p_e;
    double damp_coeff = def_damp_coeff;
    double init_obs = def_init_obs;
    double obs_damp_coeff = def_obs_damp_coeff;

    if (use_vbee)
    {
        k = read_int("k", def_k);
        n = read_int("n", def_n);
        a_th = read_double("a_th", def_a_th);
        f_th = read_double("f_th", def_f_th);
        init_p_e = read_double("init_p_e", def_init_p_e);
        damp_coeff = read_double("damp_coeff", def_damp_coeff);
        init_obs = read_double("init_obs", def_init_obs);
        obs_damp_coeff = read_double("obs_damp_coeff", def_obs_damp_coeff);
    }
    else
    {
        std::cout << "Using default parameter values." << std::endl;
    }

    try
    {
        SQLite::Statement insert(*_test_db, "INSERT INTO Params (name, use_vbee, weight_ransac, k, n, a_th, f_th, init_p_e, damp_coeff, init_obs, obs_damp_coeff) VALUES (?,?,?,?,?,?,?,?,?,?,?);");
        insert.bind(1, name);
        insert.bind(2, use_vbee ? 1 : 0);
        insert.bind(3, weight_ransac ? 1 : 0);
        insert.bind(4, k);
        insert.bind(5, n);
        insert.bind(6, a_th);
        insert.bind(7, f_th);
        insert.bind(8, init_p_e);
        insert.bind(9, damp_coeff);
        insert.bind(10, init_obs);
        insert.bind(11, obs_damp_coeff);
        insert.exec();
        std::cout << "Params saved to test DB.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to insert Params: " << e.what() << std::endl;
    }
}

void DatabaseManager::_initTestDB()
{
    std::ifstream schemaFile("./etc/test_db.schema");
    if (!schemaFile.is_open())
    {
        throw std::runtime_error("Failed to open ./etc/test_db.schema");
    }
    std::string schemaSql((std::istreambuf_iterator<char>(schemaFile)), std::istreambuf_iterator<char>());
    schemaFile.close();
    if (!schemaSql.empty())
    {
        _test_db->exec(schemaSql);
    }
}

std::vector<Trajectory> DatabaseManager::getAllTrajectories()
{
    std::vector<Trajectory> datasets;
    SQLite::Statement query(*_common_db, "SELECT * FROM Trajectories ORDER BY Id ASC;");
    while (query.executeStep())
    {
        Trajectory dataset;
        dataset.id = query.getColumn(0).getInt();
        dataset.name = query.getColumn(1).getString();
        dataset.path = query.getColumn(2).getString();
        datasets.push_back(dataset);
    }
    return datasets;
}

void DatabaseManager::addTrackTime(double time)
{
    enqueue([this, time]()
            {
        SQLite::Statement query(*_test_db, "INSERT INTO TrackTimes (time, traj) VALUES (?, ?);");
        query.bind(1, time);
        query.bind(2, traj);
        query.exec(); });
}

void DatabaseManager::addRelocTime(double time)
{
    enqueue([this, time]()
            {
        SQLite::Statement query(*_test_db, "INSERT INTO RelocTimes (time, traj) VALUES (?, ?);");
        query.bind(1, time);
        query.bind(2, traj);
        query.exec(); });
}

void DatabaseManager::addFramePose(double x, double y, double z, double r_x, double r_y, double r_z, double r_w)
{
    enqueue([=, this]()
            {
        SQLite::Statement query(*_test_db, "INSERT INTO FramePoses (timestamp, traj, x, y, z, r_x, r_y, r_z, r_w) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);");
        query.bind(1, timestamp);
        query.bind(2, traj);
        query.bind(3, x);
        query.bind(4, y);
        query.bind(5, z);
        query.bind(6, r_x);
        query.bind(7, r_y);
        query.bind(8, r_z);
        query.bind(9, r_w);
        query.exec(); });
}

void DatabaseManager::addKeyframePose(double kfTimestamp, double x, double y, double z, double r_x, double r_y, double r_z, double r_w)
{
    enqueue([=, this]()
            {
        // Use a small epsilon to find the closest timestamp
        const double epsilon = 1e-5;
        SQLite::Statement stmt(*_test_db, "SELECT traj FROM FramePoses WHERE ABS(timestamp - ?) < ? ORDER BY ABS(timestamp - ?) ASC LIMIT 1;");
        stmt.bind(1, kfTimestamp);
        stmt.bind(2, epsilon);
        stmt.bind(3, kfTimestamp);

        double kfTraj = -1;
        if (stmt.executeStep())
        {
            kfTraj = stmt.getColumn(0).getDouble();
        }
        else
        {
            std::cerr << "No matching FramePose found for timestamp " << kfTimestamp << std::endl;
            return;
        }

        SQLite::Statement query(*_test_db, "INSERT INTO KeyframePoses (timestamp, traj, x, y, z, r_x, r_y, r_z, r_w) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);");
        query.bind(1, kfTimestamp);
        query.bind(2, kfTraj);
        query.bind(3, x);
        query.bind(4, y);
        query.bind(5, z);
        query.bind(6, r_x);
        query.bind(7, r_y);
        query.bind(8, r_z);
        query.bind(9, r_w);
        query.exec(); });
}

void DatabaseManager::writeVBEELines()
{
    enqueue([this]()
            {
        if (vbeeLines.empty())
            return;

        SQLite::Transaction transaction(*_test_db);
        SQLite::Statement query(*_test_db, "INSERT INTO VBEE (mpID, timestamp, traj, p_e, est, obs, seen) VALUES (?, ?, ?, ?, ?, ?, ?);");
        for (const auto &line : vbeeLines)
        {
            query.bind(1, line.mpID);
            query.bind(2, timestamp);
            query.bind(3, traj);
            query.bind(4, line.p_e);
            query.bind(5, line.est);
            query.bind(6, line.obs);
            query.bind(7, line.seen ? 1 : 0);
            query.exec();
            query.reset();
        }
        transaction.commit();
        vbeeLines.clear(); });
}

void DatabaseManager::addVBEELine(int mpID, float p_e, float model_est, float observability, bool seen)
{
    enqueue([=, this]()
            { vbeeLines.push_back({mpID, p_e, model_est, observability, seen}); });
}

void DatabaseManager::addTrajectoryToTest(const int &dataset_id)
{
    enqueue([=, this]()
            {
        SQLite::Statement query(*_test_db, "INSERT INTO Trajectories (Id) VALUES (?);");
        query.bind(1, dataset_id);
        query.exec(); });
}

void DatabaseManager::addRANSACStats(const std::string &type, int iterations, int inliers, double time, bool success, bool refined)
{
    enqueue([=, this]()
            {
        SQLite::Statement query(*_test_db, "INSERT INTO RANSACStats (timestamp, type, iterations, inliers, time, success, refined) VALUES (?, ?, ?, ?, ?, ?, ?);");
        query.bind(1, timestamp);
        query.bind(2, type);
        query.bind(3, iterations);
        query.bind(4, inliers);
        query.bind(5, time);
        query.bind(6, success ? 1 : 0);
        query.bind(7, refined ? 1 : 0);
        query.exec(); });
}

Trajectory DatabaseManager::getTrajectoryById(int id)
{
    std::lock_guard<std::mutex> dblock(db_access_mutex);
    Trajectory traj{.id = -1, .name = "", .path = ""};
    SQLite::Statement query(*_common_db, "SELECT Id, Name, Path FROM Trajectories WHERE Id = ?;");
    query.bind(1, id);
    if (query.executeStep())
    {
        traj.id = query.getColumn(0).getInt();
        traj.name = query.getColumn(1).getString();
        traj.path = query.getColumn(2).getString();
    }
    return traj;
}

std::vector<int> DatabaseManager::getTrajectoryIDs() const
{
    std::lock_guard<std::mutex> dblock(db_access_mutex);
    std::vector<int> ids;
    SQLite::Statement query(*_test_db, "SELECT Id FROM Trajectories ORDER BY [Order] ASC;");
    while (query.executeStep())
    {
        ids.push_back(query.getColumn(0).getInt());
    }
    return ids;
}

DBParams DatabaseManager::getParams() const
{
    std::lock_guard<std::mutex> dblock(db_access_mutex);
    DBParams p{};
    SQLite::Statement query(*_test_db, "SELECT use_vbee, weight_ransac, k, n, a_th, f_th, init_p_e, damp_coeff, init_obs, obs_damp_coeff FROM Params LIMIT 1;");
    if (query.executeStep())
    {
        p.use_vbee = query.getColumn(0).getInt() == 1;
        p.weight_ransac = query.getColumn(1).getInt() == 1;
        p.k = query.getColumn(2).getInt();
        p.n = query.getColumn(3).getInt();
        p.a_th = static_cast<float>(query.getColumn(4).getDouble());
        p.f_th = static_cast<float>(query.getColumn(5).getDouble());
        p.init_p_e = static_cast<float>(query.getColumn(6).getDouble());
        p.damp_coeff = static_cast<float>(query.getColumn(7).getDouble());
        p.init_obs = static_cast<float>(query.getColumn(8).getDouble());
        p.obs_damp_coeff = static_cast<float>(query.getColumn(9).getDouble());
    }
    else
    {
        throw std::runtime_error("No Params row found in test DB");
    }
    return p;
}
