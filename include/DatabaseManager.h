#pragma once

#include <SQLiteCpp/SQLiteCpp.h>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <functional>

typedef struct
{
    int id;
    std::string name;
    std::string path;
} Trajectory;

typedef struct
{
    bool use_vbee;
    bool weight_ransac;
    int k;
    int n;
    float a_th;
    float f_th;
    float init_p_e;
    float damp_coeff;
    float init_obs;
    float obs_damp_coeff;
} DBParams;

typedef struct
{
    int mpID;
    float p_e;
    float est;
    float obs;
    bool seen;
} VBEELine;

class DatabaseManager
{
public:
    ~DatabaseManager();
    // Singleton access
    static DatabaseManager &Instance();
    static void Init(const std::string &test_name);

    // Delete copy/move
    DatabaseManager(const DatabaseManager &) = delete;
    DatabaseManager &operator=(const DatabaseManager &) = delete;
    DatabaseManager(DatabaseManager &&) = delete;
    DatabaseManager &operator=(DatabaseManager &&) = delete;
    // Returns true if the worker queue is empty and not processing a task
    bool workerIsDone() const;
    std::vector<int> getTrajectoryIDs() const;

    void addFramePose(double x, double y, double z, double r_x, double r_y, double r_z, double r_w);
    void addKeyframePose(double kfTimestamp, double x, double y, double z, double r_x, double r_y, double r_z, double r_w);

    void addRANSACStats(const std::string &type, int iterations, int inliers, double time, bool success, bool refined);

    std::vector<VBEELine> vbeeLines;
    void addVBEELine(int mpID, float p_e, float model_est, float observability, bool seen);
    void addTrackTime(double time);
    void addRelocTime(double time);
    void writeVBEELines();

    void setTimestamp(double ts)
    {
        enqueue([this, ts]()
                {
            writeVBEELines();
            timestamp = ts; });
    }
    void nextTrajectory()
    {
        enqueue([this]()
                {
            writeVBEELines();
            traj++; });
    }

    Trajectory getTrajectoryById(int id);
    DBParams getParams() const;

protected:
    DatabaseManager(const std::string &test_name);

private:
    DatabaseManager() = default;
    static std::unique_ptr<DatabaseManager> _instance;

    void _initTestDB();
    void _initCommonDB();
    void _interactivePopulateTestDB();
    void _clearTestData();

    int _getTrajAtTS(double ts) const;

    std::vector<Trajectory> getAllTrajectories();

    void addTrajectoryToTest(const int &dataset_id);

    std::unique_ptr<SQLite::Database> _test_db;
    std::unique_ptr<SQLite::Database> _common_db;

    double timestamp = 0.0;
    int traj = 0;

    // Async machinery
    using Task = std::function<void()>;
    mutable std::mutex queue_mutex;
    std::queue<Task> task_queue;
    std::condition_variable queue_cv;
    std::thread worker_thread;
    bool stop_worker = false;

    // Mutex for all DB access (getters and worker thread)
    mutable std::mutex db_access_mutex;

    void enqueue(Task task);
    void workerLoop();
};
