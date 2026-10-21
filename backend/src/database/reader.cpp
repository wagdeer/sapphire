#include "backend/database/reader.hpp"

#include <Eigen/Cholesky>
#include <backend/common/common_type.hpp>
#include <backend/tools/filesystem.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/strings.hpp>
#include <chrono>
#include <thread>

#include "backend/common/state_group.hpp"
#include "backend/database/compression.hpp"
#include "backend/database/driver.hpp"
#include "backend/database/sensor_archive.hpp"
#include "backend/sensor/camera/utils.hpp"

namespace sapphire::backend {
namespace {

cv::Point3f transformCvPoint(const cv::Point3f &point, const StateGroupf &state) {
  const StateGroupf::Vector3 transformed = state.transformPoint(StateGroupf::Vector3(point.x, point.y, point.z));
  return cv::Point3f(transformed.x(), transformed.y(), transformed.z());
}

}  // namespace

DBReader::DBReader(const std::string &databasePath, float frameRate, bool odometryIgnored, bool ignoreGoalDelay, bool goalsIgnored, int startId,
                   const std::vector<unsigned int> &cameraIndices, int stopId, bool intermediateNodesIgnored, bool featuresIgnored, int startMapId,
                   int stopMapId, bool priorsIgnored, bool intermediateNodesAreNormalNodes,
                   const std::vector<StateGroupf> &cameraLocalTransformOverrides)
    : _paths(sapphire::backend::tools::split(databasePath, ';')),
      _frameRate(frameRate),
      _odometryIgnored(odometryIgnored),
      _ignoreGoalDelay(ignoreGoalDelay),
      _goalsIgnored(goalsIgnored),
      _startId(startId),
      _stopId(stopId),
      _cameraIndices(cameraIndices),
      _intermediateNodesIgnored(intermediateNodesIgnored),
      _intermediateNodesAreNormalNodes(intermediateNodesAreNormalNodes),
      _featuresIgnored(featuresIgnored),
      _priorsIgnored(priorsIgnored),
      _startMapId(startMapId),
      _stopMapId(stopMapId),
      _cameraLocalTransformOverrides(cameraLocalTransformOverrides),
      _dbDriver(0),
      _currentId(_ids.end()),
      _previousMapId(-1),
      _previousStamp(0),
      _previousMapID(0) {
  checkArguments();
}

DBReader::DBReader(const std::list<std::string> &databasePaths, float frameRate, bool odometryIgnored, bool ignoreGoalDelay, bool goalsIgnored,
                   int startId, const std::vector<unsigned int> &cameraIndices, int stopId, bool intermediateNodesIgnored, bool featuresIgnored,
                   int startMapId, int stopMapId, bool priorsIgnored, bool intermediateNodesAreNormalNodes,
                   const std::vector<StateGroupf> &cameraLocalTransformOverrides)
    : _paths(databasePaths.begin(), databasePaths.end()),
      _frameRate(frameRate),
      _odometryIgnored(odometryIgnored),
      _ignoreGoalDelay(ignoreGoalDelay),
      _goalsIgnored(goalsIgnored),
      _startId(startId),
      _stopId(stopId),
      _cameraIndices(cameraIndices),
      _intermediateNodesIgnored(intermediateNodesIgnored),
      _intermediateNodesAreNormalNodes(intermediateNodesAreNormalNodes),
      _featuresIgnored(featuresIgnored),
      _priorsIgnored(priorsIgnored),
      _startMapId(startMapId),
      _stopMapId(stopMapId),
      _cameraLocalTransformOverrides(cameraLocalTransformOverrides),
      _dbDriver(0),
      _currentId(_ids.end()),
      _previousMapId(-1),
      _previousStamp(0),
      _previousMapID(0) {
  checkArguments();
}

void DBReader::checkArguments() {
  if (_stopId > 0 && _stopId < _startId) {
    _stopId = _startId;
  }

  if (_stopMapId > -1 && _stopMapId < _startMapId) {
    _stopMapId = _startMapId;
  }

  if (!_cameraLocalTransformOverrides.empty()) {
    if (!_cameraIndices.empty() && _cameraIndices.size() != _cameraLocalTransformOverrides.size()) {
      EXT_LOG_ERROR("Camera local transform overrides ({:d}) are not the same size than the camera indices ({:d}). The overrides are ignored.",
                    (int)_cameraLocalTransformOverrides.size(), (int)_cameraIndices.size());
      _cameraLocalTransformOverrides.clear();
    }
    for (size_t i = 0; i < _cameraLocalTransformOverrides.size(); ++i) {
      if (_cameraLocalTransformOverrides[i].isNull()) {
        EXT_LOG_ERROR("Camera local transform overrides vector cannot contains null transforms! Clearing overrides.");
        _cameraLocalTransformOverrides.clear();
        break;
      }
    }
  }
}

DBReader::~DBReader() {
  if (_dbDriver) {
    _dbDriver->closeConnection();
    delete _dbDriver;
  }
}

bool DBReader::init() {
  if (_dbDriver) {
    _dbDriver->closeConnection();
    delete _dbDriver;
    _dbDriver = 0;
  }
  _ids.clear();
  _currentId = _ids.end();
  _previousMapId = -1;
  _previousInfMatrix.setIdentity();
  _previousStamp = 0;
  _previousMapID = 0;

  if (_paths.size() == 0) {
    EXT_LOG_ERROR("No database path set...");
    return false;
  }

  std::string path = _paths.front();
  if (!tools::path_exists(path)) {
    EXT_LOG_ERROR("Database path does not exist ({})", path.c_str());
    return false;
  }

  _dbDriver = DBDriver::create({{"database.sqlite.in_memory", "false"}});
  if (!_dbDriver) {
    EXT_LOG_ERROR("Driver doesn't exist.");
    return false;
  }
  if (!_dbDriver->openConnection(path)) {
    EXT_LOG_ERROR("Can't open database {}", path.c_str());
    delete _dbDriver;
    _dbDriver = 0;
    return false;
  }

  _dbDriver->getAllNodeIds(_ids);
  _currentId = _ids.begin();
  if (_startId > 0 && _ids.size()) {
    std::set<int>::iterator iter = _ids.find(_startId);
    if (iter == _ids.end()) {
      EXT_LOG_WARN("Start index is too high ({:d}), the last ID in database is {:d}. Starting from beginning...", _startId, *_ids.rbegin());
    } else {
      _currentId = iter;
    }
  }

  _timer.reset();

  return true;
}

SensorData DBReader::readNext(DBReaderInfo *info) {
  if (info) *info = DBReaderInfo{};

  if (_frameRate > 0.0f) {
    const double remaining = 1.0 / static_cast<double>(_frameRate) - _timer.elapsed_seconds();
    if (remaining > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
    _timer.reset();
  }

  SensorData data = readCurrent(info);
  if (data.id() > 0 && _stopId > 0 && data.id() > _stopId) {
    EXT_LOG_INFO("Last ID {:d} has been reached! Ignoring", _stopId);
    return SensorData();
  }
  if (data.id() == 0) {
    EXT_LOG_INFO("no more images...");
    while (_paths.size() > 1 && data.id() == 0) {
      _paths.erase(_paths.begin());
      EXT_LOG_WARN("Loading next database \"{}\"...", _paths.front().c_str());
      if (!this->init()) {
        EXT_LOG_ERROR("Failed to initialize the next database \"{}\"", _paths.front().c_str());
        return data;
      } else {
        data = readCurrent(info);
      }
    }
  }

  if (data.id()) {
    if (data.stamp() == 0) {
      data.setStamp(tools::wall_time_seconds());
    }
  }
  return data;
}

SensorData DBReader::readCurrent(DBReaderInfo *info) {
  SensorData data;
  if (_dbDriver) {
    while (_currentId != _ids.end()) {
      std::list<int> signIds;
      signIds.push_back(*_currentId);
      std::list<Node *> nodes;
      _dbDriver->loadNodes(signIds, nodes);
      if (nodes.empty()) {
        return data;
      }
      Node *s = nodes.front();

      if (_intermediateNodesIgnored && s->getWeight() == -1) {
        EXT_LOG_DEBUG("Ignoring node {:d} (intermediate nodes ignored)", s->id());
        ++_currentId;
        delete s;
        continue;
      }

      if (s->mapId() < _startMapId || (_stopMapId >= 0 && s->mapId() > _stopMapId)) {
        EXT_LOG_DEBUG("Ignoring node {:d} (map id={:d}, min={:d} max={:d})", s->id(), s->mapId(), _startMapId, _stopMapId);
        ++_currentId;
        delete s;
        continue;
      }

      SensorRecord record;
      _dbDriver->getNodeData(s->id(), record);
      decodeSensorData(record);
      data = std::move(record.observation);
      data.setGlobalDescriptors(s->featmap().global_descriptors());

      // info
      StateGroupf pose = s->getPose();
      StateGroupf globalPose;
      std::optional<Mat66_t<double>> globalPoseCov;

      std::multimap<int, Link> priorLinks;
      if (!_priorsIgnored) {
        _dbDriver->loadLinks(*_currentId, priorLinks, Link::kPosePrior);
        if (priorLinks.size() && !priorLinks.begin()->second.transform().isNull()) {
          globalPose = priorLinks.begin()->second.transform();
          globalPoseCov = priorLinks.begin()->second.infMatrix().llt().solve(Mat66_t<double>::Identity());
        }
      }

      Mat66_t<double> infMatrix = Mat66_t<double>::Identity();
      if (!_odometryIgnored) {
        std::multimap<int, Link> links;
        _dbDriver->loadLinks(*_currentId, links, Link::kOdometry);
        if (links.size() && links.begin()->first < *_currentId) {
          // assume the first is the backward neighbor, take its variance
          infMatrix = links.begin()->second.infMatrix();
          _previousInfMatrix = infMatrix;
        } else {
          if (_previousMapId != s->mapId()) {
            // first node, set high variance to make rtabmap trigger a new map
            infMatrix /= 9999.0;
            EXT_LOG_DEBUG("First node of map {:d}, variance set to 9999", s->mapId());
          } else {
            // In case the graph was reduced, look for forward neighbor link from previous id
            bool covAdded = false;
            if (_currentId != _ids.begin()) {
              std::set<int>::iterator previousId = _currentId;
              --previousId;
              std::multimap<int, Link> previousLinks;
              _dbDriver->loadLinks(*previousId, previousLinks, Link::kOdometry);
              if (previousLinks.size() && previousLinks.rbegin()->first == *_currentId) {
                // assume the last is the forward neighbor pointing to current ID, take its covariance
                infMatrix = previousLinks.rbegin()->second.infMatrix();
                _previousInfMatrix = infMatrix;
                covAdded = true;
              }
            }

            if (!covAdded) {
              // A persisted pose prior carries the covariance when no neighbor link is available.
              _dbDriver->loadLinks(*_currentId, links, Link::kPosePrior);
              if (links.size()) {
                // assume the first is the backward neighbor, take its variance
                infMatrix = links.begin()->second.infMatrix();
                _previousInfMatrix = infMatrix;
              } else {
                // we have a node not linked to map, use last variance
                EXT_LOG_WARN("The node loaded ({:d}) doesn't have neighbor, re-using the covariance of the previous link for odometry.", s->id());
                infMatrix = _previousInfMatrix;
              }
            }
          }
        }
        _previousMapId = s->mapId();
      } else {
        pose = StateGroupf();
      }

      int seq = *_currentId;
      ++_currentId;

      // Frame rate
      if (_frameRate < 0.0f) {
        if (s->getStamp() == 0) {
          EXT_LOG_ERROR("The option to use database stamps is set (framerate<0), but there are no stamps saved in the database! Aborting...");
          delete s;
          return data;
        } else if (_previousMapID == s->mapId() && _previousStamp > 0) {
          float ratio = -_frameRate;
          int sleepTime = 1000.0 * (s->getStamp() - _previousStamp) / ratio - 1000.0 * _timer.elapsed_seconds();
          double stamp = s->getStamp();
          if (sleepTime > 10000) {
            EXT_LOG_WARN("Detected long delay ({:d} sec, stamps = {:.6f} vs {:.6f}). Waiting a maximum of 10 seconds.", sleepTime / 1000,
                         _previousStamp, s->getStamp());
            sleepTime = 10000;
            stamp = _previousStamp + 10;
          }
          if (sleepTime > 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime - 2));
          }

          // Add precision at the cost of a small overhead
          while (_timer.elapsed_seconds() < (stamp - _previousStamp) / ratio - 0.000001) {
            //
          }

          double slept = _timer.elapsed_seconds();
          _timer.reset();
          EXT_LOG_DEBUG("slept={:.6f}s vs target={:.6f}s (ratio={:.6f})", slept, (stamp - _previousStamp) / ratio, ratio);
        }
        _previousStamp = s->getStamp();
        _previousMapID = s->mapId();
      }

      std::map<int, int> cameraOldNewIndices;
      std::vector<camera::Perspective> dbModels = data.perspectives();

      if (!_cameraLocalTransformOverrides.empty() && !_cameraIndices.empty() && _cameraIndices.size() != _cameraLocalTransformOverrides.size()) {
        EXT_LOG_ERROR("Camera local transform overrides ({:d}) are not the same size than the camera indices ({:d}). The overrides are ignored.",
                      (int)_cameraLocalTransformOverrides.size(), (int)_cameraIndices.size());
        _cameraLocalTransformOverrides.clear();
      }

      std::vector<StateGroupf> combinedLocalTransforms;
      if (dbModels.size() > 1 && !_cameraIndices.empty()) {
        // update images and local transforms
        const std::vector<cv::Mat> dbImages = camera::splitCameraMosaic(data.imageRaw(), dbModels);
        const std::vector<cv::Mat> dbDepthImages = camera::splitCameraMosaic(data.depthRaw(), dbModels);
        std::vector<cv::Mat> combinedImages;
        std::vector<cv::Mat> combinedDepthImages;
        std::vector<camera::Perspective> combinedModels;
        for (size_t i = 0; i < _cameraIndices.size(); ++i) {
          EXT_ASSERT_MSG(_cameraIndices[i] < dbModels.size(), "DBReader: camera index {:d} is not valid (should be between 0 and {:d})",
                         (long)_cameraIndices[i], dbModels.size() - 1);

          combinedImages.push_back(dbImages[_cameraIndices[i]]);
          if (!dbDepthImages.empty()) {
            combinedDepthImages.push_back(dbDepthImages[_cameraIndices[i]]);
          }

          camera::Perspective model = data.perspectives()[_cameraIndices[i]];
          if (!_cameraLocalTransformOverrides.empty()) {
            model.setCameraToBase(_cameraLocalTransformOverrides[i].compose(camera::Perspective::opticalRotation()));
          }
          combinedModels.push_back(model);
          combinedLocalTransforms.push_back(model.cameraToBase());
          cameraOldNewIndices.insert(std::make_pair(_cameraIndices[i], i));
        }
        data.setImageData(camera::makeCameraMosaic(combinedImages), camera::makeCameraMosaic(combinedDepthImages), combinedModels);
      } else if (!_cameraLocalTransformOverrides.empty() && _cameraLocalTransformOverrides.size() == dbModels.size()) {
        // just update local transforms
        std::vector<camera::Perspective> combinedModels;
        for (size_t i = 0; i < dbModels.size(); ++i) {
          camera::Perspective model = data.perspectives()[i];
          model.setCameraToBase(_cameraLocalTransformOverrides[i].compose(camera::Perspective::opticalRotation()));
          combinedModels.push_back(model);
          combinedLocalTransforms.push_back(model.cameraToBase());
        }
        data.setPerspectives(combinedModels);
      }
      data.setId(!_intermediateNodesAreNormalNodes && s->getWeight() == -1 ? -1 : seq);
      data.setStamp(s->getStamp());
      if (!globalPose.isNull()) {
        data.setGlobalPose(globalPose, globalPoseCov);
      }

      EXT_LOG_DEBUG("RGB={:d} Depth={:d}, Grid={:d}, UserData={:d}, GlobalPose={:d}", data.imageRaw().empty() ? 0 : 1,
                    data.depthRaw().empty() ? 0 : 1, !data.localGrid() ? 0 : 1, data.userDataRaw().empty() ? 0 : 1, globalPose.isNull() ? 0 : 1);

      // The node stays alive until every output branch has shared or detached
      // its measurements. Merely inspecting/selecting rows needs no clone.
      const cv::Mat &descriptors = s->getWordsDescriptors();
      const std::vector<cv::KeyPoint> &keypoints = s->getWordsKpts();
      const std::vector<cv::Point3f> &keypoints3D = s->getWords3();
      if (!_featuresIgnored && !keypoints.empty() && (keypoints3D.empty() || keypoints.size() == keypoints3D.size()) &&
          (descriptors.empty() || (int)keypoints.size() == descriptors.rows)) {
        if (!cameraOldNewIndices.empty()) {
          cv::Mat newDescriptors;
          std::vector<cv::KeyPoint> newKeypoints;
          std::vector<cv::Point3f> newKeypoints3D;
          const std::vector<cv::Rect> dbCameraRois = camera::cameraImageRois(dbModels);
          const std::vector<cv::Rect> combinedCameraRois = camera::cameraImageRois(data.perspectives());
          for (size_t i = 0; i < keypoints.size(); ++i) {
            const int cameraIndex = camera::cameraIndexForPoint(keypoints.at(i).pt, dbCameraRois);
            EXT_ASSERT_MSG(cameraIndex >= 0 && cameraIndex < (int)dbModels.size(), "cameraIndex={:d}, db models={:d}, kpt=({:.6f},{:.6f})",
                           cameraIndex, (int)dbModels.size(), keypoints[i].pt.x, keypoints[i].pt.y);
            if (cameraOldNewIndices.find(cameraIndex) != cameraOldNewIndices.end()) {
              int newCameraIndex = cameraOldNewIndices.at(cameraIndex);
              newKeypoints.push_back(keypoints[i]);
              newKeypoints.back().pt.x = newKeypoints.back().pt.x - dbCameraRois[cameraIndex].x + combinedCameraRois[newCameraIndex].x;
              if (!keypoints3D.empty()) {
                cv::Point3f pt = transformCvPoint(keypoints3D.at(i), dbModels[cameraIndex].cameraToBase().inverse());
                pt = transformCvPoint(pt, combinedLocalTransforms[newCameraIndex]);
                newKeypoints3D.push_back(pt);
              }
              if (!descriptors.empty()) {
                newDescriptors.push_back(descriptors.row(i));
              }
            }
          }
          data.visualFrame().set_features_owned(std::move(newKeypoints), std::move(newKeypoints3D), std::move(newDescriptors));
        } else if (!combinedLocalTransforms.empty()) {
          // We are overriding the camera local transforms, let's move 3D words accordingly
          EXT_ASSERT(dbModels.size() == combinedLocalTransforms.size());
          std::vector<cv::Point3f> newKeypoints3D;
          const std::vector<cv::Rect> dbCameraRois = camera::cameraImageRois(dbModels);
          for (size_t i = 0; i < keypoints3D.size(); ++i) {
            const int cameraIndex = camera::cameraIndexForPoint(keypoints.at(i).pt, dbCameraRois);
            EXT_ASSERT_MSG(cameraIndex >= 0 && cameraIndex < (int)dbModels.size(), "cameraIndex={:d}, db models={:d}, kpt=({:.6f},{:.6f})",
                           cameraIndex, (int)dbModels.size(), keypoints[i].pt.x, keypoints[i].pt.y);
            cv::Point3f pt = transformCvPoint(keypoints3D.at(i), dbModels[cameraIndex].cameraToBase().inverse());
            pt = transformCvPoint(pt, combinedLocalTransforms[cameraIndex]);
            newKeypoints3D.push_back(pt);
          }
          data.visualFrame().set_features_owned(std::vector<cv::KeyPoint>(keypoints), std::move(newKeypoints3D), descriptors.clone());
        } else {
          // Retain the same immutable measurements without copying camera metadata
          // or replacing caller-selected calibration/image bounds.
          if (s->loop_features())
            data.visualFrame().share_features(s->loop_features());
          else
            data.visualFrame().setFeatures(keypoints, keypoints3D, descriptors);
        }
      } else if (!_featuresIgnored && !keypoints.empty() && (!keypoints3D.empty() || !descriptors.empty())) {
        EXT_LOG_ERROR("Missing feature data, features won't be published.");
      }

      if (data.imageRaw().empty() && s->getWeight() >= 0 && keypoints.empty()) {
        EXT_LOG_WARN("No image loaded from the database for id={:d}!", seq);
      }

      if (!_odometryIgnored) {
        if (pose.isNull()) {
          EXT_LOG_WARN(
              "Reading the database: odometry is null! "
              "Please set \"Ignore odometry = true\" if there is "
              "no odometry in the database.");
        }
        if (info) {
          info->odomPose = pose;
          EXT_ASSERT(infMatrix.allFinite());
          info->odomCovariance = infMatrix.llt().solve(Mat66_t<double>::Identity());
          info->odomVelocity = s->getVelocity();
          EXT_LOG_DEBUG("odom variance = {:.6f}/{:.6f}", info->odomCovariance(0, 0), info->odomCovariance(5, 5));
        }
      }
      delete s;
      break;
    }
  } else {
    EXT_LOG_ERROR("Not initialized...");
  }
  return data;
}

} /* namespace sapphire::backend */
