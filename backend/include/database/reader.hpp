#ifndef DBREADER_H_
#define DBREADER_H_

#include <backend/common/sensor_data.hpp>
#include <backend/common/state_group.hpp>
#include <backend/tools/timer.hpp>
#include <list>
#include <opencv2/core/core.hpp>
#include <set>
#include <vector>

namespace sapphire::backend {

class DBDriver;

struct DBReaderInfo {
  StateGroupf odomPose;
  Mat66_t<double> odomCovariance = Mat66_t<double>::Identity();
  std::vector<float> odomVelocity;
};

class DBReader {
 public:
  DBReader(const std::string& databasePath,
           float frameRate = 0.0f,  // -1 = use Database stamps, 0 = inf
           bool odometryIgnored = false, bool ignoreGoalDelay = false, bool goalsIgnored = false, int startId = 0,
           const std::vector<unsigned int>& cameraIndices = std::vector<unsigned int>(), int stopId = 0, bool intermediateNodesIgnored = false,
           bool featuresIgnored = false, int startMapId = 0, int stopMapId = -1, bool priorsIgnored = false,
           bool intermediateNodesAreNormalNodes = false, const std::vector<StateGroupf>& cameraLocalTransformOverrides = std::vector<StateGroupf>());
  DBReader(const std::list<std::string>& databasePaths,
           float frameRate = 0.0f,  // -1 = use Database stamps, 0 = inf
           bool odometryIgnored = false, bool ignoreGoalDelay = false, bool goalsIgnored = false, int startId = 0,
           const std::vector<unsigned int>& cameraIndices = std::vector<unsigned int>(), int stopId = 0, bool intermediateNodesIgnored = false,
           bool featuresIgnored = false, int startMapId = 0, int stopMapId = -1, bool priorsIgnored = false,
           bool intermediateNodesAreNormalNodes = false, const std::vector<StateGroupf>& cameraLocalTransformOverrides = std::vector<StateGroupf>());
  ~DBReader();

  bool init();
  SensorData readNext(DBReaderInfo* info = nullptr);

  const DBDriver* driver() const { return _dbDriver; }

 private:
  SensorData readCurrent(DBReaderInfo* info);
  void checkArguments();

 private:
  std::vector<std::string> _paths;
  float _frameRate;
  bool _odometryIgnored;
  bool _ignoreGoalDelay;
  bool _goalsIgnored;
  int _startId;
  int _stopId;
  std::vector<unsigned int> _cameraIndices;
  bool _intermediateNodesIgnored;
  bool _intermediateNodesAreNormalNodes;
  bool _featuresIgnored;
  bool _priorsIgnored;
  int _startMapId;
  int _stopMapId;
  std::vector<StateGroupf> _cameraLocalTransformOverrides;

  DBDriver* _dbDriver;
  tools::Timer _timer;
  std::set<int> _ids;
  std::set<int>::iterator _currentId;
  int _previousMapId;
  Mat66_t<double> _previousInfMatrix = Mat66_t<double>::Identity();
  double _previousStamp;
  int _previousMapID;
};

} /* namespace sapphire::backend */
#endif /* DBREADER_H_ */
