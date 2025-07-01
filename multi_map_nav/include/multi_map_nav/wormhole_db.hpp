#pragma once
#include <string>
#include <sqlite3.h>

class WormholeDB {
public:
  WormholeDB(const std::string &db_path);
  ~WormholeDB();
  bool saveWormhole(const std::string &source_map,
                    const std::string &target_map,
                    double x, double y, double theta);
  bool getWormhole(const std::string &source_map, const std::string &target_map, double &x, double &y, double &theta);


private:
  sqlite3 *db_;
};
