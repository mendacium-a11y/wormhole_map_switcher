#pragma once
#include <string>
#include <sqlite3.h>

class PoseSaverDB {
public:
  PoseSaverDB(const std::string &db_path);
  ~PoseSaverDB();
  bool savePose(const std::string &map_name, const std::string &point_name, double x, double y, double theta);
  bool getSavedPose(const std::string &map_name, const std::string &point_name, double &x, double &y, double &theta);


private:
  sqlite3 *db_;
};
