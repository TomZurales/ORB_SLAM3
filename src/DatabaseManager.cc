#include "DatabaseManager.h"

DatabaseManager::DatabaseManager(const std::string &dbPath)
{
    _db = std::make_unique<SQLite::Database>(dbPath, SQLite::OPEN_READWRITE);
}

void DatabaseManager::_initDB()
{
    // Params Table
    _db->exec(
        "CREATE TABLE 'Params' ("
        "'k' INTEGER NOT NULL,"
        "'n' INTEGER NOT NULL,"
        "'a_th' REAL NOT NULL,"
        "'f_th' REAL NOT NULL,"
        "'init_p_e' REAL NOT NULL,"
        "'damp_coeff' REAL NOT NULL,"
        "'init_obs' REAL NOT NULL,"
        "'obs_damp_coeff' REAL NOT NULL"
        ");");

    _db->exec(
        "CREATE TABLE 'Trajectories' ("
        "'Id'	INTEGER NOT NULL UNIQUE,"
        "'Path'	TEXT NOT NULL,"
        "PRIMARY KEY('Id' AUTOINCREMENT)"
        ");");
}