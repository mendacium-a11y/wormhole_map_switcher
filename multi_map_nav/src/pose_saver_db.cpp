#include "multi_map_nav/pose_saver_db.hpp"
#include <iostream>

PoseSaverDB::PoseSaverDB(const std::string &db_path) {
  if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
    std::cerr << "Failed to open DB: " << sqlite3_errmsg(db_) << std::endl;
    db_ = nullptr;
  } else {
    const char *create_stmt = R"(
      CREATE TABLE IF NOT EXISTS saved_poses (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        map_name TEXT,
        point_name TEXT,
        x REAL,
        y REAL,
        theta REAL
      );
    )";
    char *errmsg;
    sqlite3_exec(db_, create_stmt, nullptr, nullptr, &errmsg);
  }
}

PoseSaverDB::~PoseSaverDB() {
  if (db_) sqlite3_close(db_);
}

bool PoseSaverDB::savePose(const std::string &map_name, const std::string &point_name, double x, double y, double theta) {
  if (!db_) return false;

  const char *insert = "INSERT INTO saved_poses (map_name, point_name, x, y, theta) VALUES (?, ?, ?, ?, ?);";
  sqlite3_stmt *stmt;
  sqlite3_prepare_v2(db_, insert, -1, &stmt, nullptr);
  sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, point_name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_double(stmt, 3, x);
  sqlite3_bind_double(stmt, 4, y);
  sqlite3_bind_double(stmt, 5, theta);



  int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  return rc == SQLITE_DONE;
}

bool PoseSaverDB::getSavedPose(const std::string &map_name, const std::string &point_name, double &x, double &y, double &theta) {
  if (!db_) return false;

  const char *query = "SELECT x, y, theta FROM saved_poses WHERE map_name = ? AND point_name = ? LIMIT 1;";
  sqlite3_stmt *stmt;
  sqlite3_prepare_v2(db_, query, -1, &stmt, nullptr);
  sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, point_name.c_str(), -1, SQLITE_TRANSIENT);

  int rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    x = sqlite3_column_double(stmt, 0);
    y = sqlite3_column_double(stmt, 1);
    theta = sqlite3_column_double(stmt, 2);
    sqlite3_finalize(stmt);
    return true;
  } else {
    sqlite3_finalize(stmt);
    return false;
  }
}
