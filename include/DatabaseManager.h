#pragma once

#include <SQLiteCpp/SQLiteCpp.h>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>

typedef struct
{
    int id;
    std::string name;
    std::string path;
} Trajectory;

typedef struct
{
    bool use_vbee;
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
    // Singleton access
    static DatabaseManager &Instance();
    static void Init(const std::string &test_name);

    // Delete copy/move
    DatabaseManager(const DatabaseManager &) = delete;
    DatabaseManager &operator=(const DatabaseManager &) = delete;
    DatabaseManager(DatabaseManager &&) = delete;
    DatabaseManager &operator=(DatabaseManager &&) = delete;

    std::vector<int> getTrajectoryIDs() const;

    void addFramePose(double x, double y, double z, double r_x, double r_y, double r_z, double r_w);
    void addKeyframePose(double kfTimestamp, double x, double y, double z, double r_x, double r_y, double r_z, double r_w);

    std::vector<VBEELine> vbeeLines;
    void addVBEELine(int mpID, float p_e, float model_est, float observability, bool seen);
    void writeVBEELines();

    void setTimestamp(double ts)
    {
        writeVBEELines();
        timestamp = ts;
    }
    void nextTrajectory()
    {
        writeVBEELines();
        traj++;
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
    std::vector<Trajectory> getAllTrajectories();

    void addTrajectoryToTest(const int &dataset_id);

    std::unique_ptr<SQLite::Database> _test_db;
    std::unique_ptr<SQLite::Database> _common_db;

    double timestamp = 0.0;
    int traj = 0;
};
