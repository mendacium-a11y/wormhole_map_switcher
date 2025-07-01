#include "multi_map_nav/wormhole_db.hpp"
#include <iostream>

WormholeDB::WormholeDB(const std::string &db_path) {
  if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
    std::cerr << "Can't open DB: " << sqlite3_errmsg(db_) << std::endl;
    db_ = nullptr;
  } else {
    const char *create_table = R"(
      CREATE TABLE IF NOT EXISTS wormholes (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        source_map TEXT,
        target_map TEXT,
        x REAL,
        y REAL,
        theta REAL
      );
    )";
    char *errMsg = nullptr;
    sqlite3_exec(db_, create_table, nullptr, nullptr, &errMsg);
  }
}

WormholeDB::~WormholeDB() {
  if (db_) sqlite3_close(db_);
}

bool WormholeDB::saveWormhole(const std::string &source_map,
                               const std::string &target_map,
                               double x, double y, double theta) {
  if (!db_) return false;

  const char *insert_sql = "INSERT INTO wormholes (source_map, target_map, x, y, theta) VALUES (?, ?, ?, ?, ?);";
  sqlite3_stmt *stmt;
  sqlite3_prepare_v2(db_, insert_sql, -1, &stmt, nullptr);
  sqlite3_bind_text(stmt, 1, source_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_double(stmt, 3, x);
  sqlite3_bind_double(stmt, 4, y);
  sqlite3_bind_double(stmt, 5, theta);

  int rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  return rc == SQLITE_DONE;
}

bool WormholeDB::getWormhole(const std::string &source_map, const std::string &target_map, double &x, double &y, double &theta) {
  if (!db_) return false;

  const char *query = "SELECT x, y, theta FROM wormholes WHERE source_map = ? AND target_map = ? LIMIT 1;";
  sqlite3_stmt *stmt;
  sqlite3_prepare_v2(db_, query, -1, &stmt, nullptr);
  sqlite3_bind_text(stmt, 1, source_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);

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

