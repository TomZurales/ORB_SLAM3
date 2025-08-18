#pragma once

#include <memory>
#include <SQLiteCpp/SQLiteCpp.h>
#include <string>
#include <iostream>

class DatabaseManager
{
public:
    DatabaseManager(const std::string &dbPath);

    std::string getNumTrajectories() const
    {
        return std::to_string(_db->execAndGet("SELECT COUNT(*) FROM Trajectories;").getInt64());
    }

private:
    void _initDB();
    std::unique_ptr<SQLite::Database> _db;
    bool _dbCreated;
};

class DatabaseManagerUser
{
    private:
        std::shared_ptr<DatabaseManager> _dbManager;
    public:
        void setDatabaseManager(std::shared_ptr<DatabaseManager> dbManager) { _dbManager = std::move(dbManager);
        }
};