#ifndef DBDRIVERSQLITE3_H_
#define DBDRIVERSQLITE3_H_
#include <backend/common/common_type.hpp>
#include <backend/params/database.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "backend/common/state_group.hpp"
#include "backend/database/driver.hpp"

typedef struct sqlite3_stmt sqlite3_stmt;
typedef struct sqlite3 sqlite3;

namespace sapphire::backend {

/**
 * @class DBDriverSqlite3
 * @brief SQLite3 implementation of @ref DBDriver for RTAB-Map map databases.
 *
 * This is the default driver returned by @ref DBDriver::create(). It stores nodes,
 * links, visual words, statistics and sensor payloads in a single \c .db file using
 * the SQLite C API.
 *
 * **Storage modes**
 * - **File-backed** (default): the database is read/written directly on disk at @ref getUrl().
 * - **In-memory**: when @ref isInMemory() is true, SQLite uses \c :memory: and the file at
 *   @ref getUrl() (if any) is loaded at open and optionally saved on @ref closeConnection().
 *
 * SQLite tuning is validated at construction and applied when connecting.
 * Keys and defaults live in params/database.hpp; an open driver is not reconfigured.
 *
 * @see DBDriver
 * @see DBDriver::create()
 */
class DBDriverSqlite3 : public DBDriver {
 public:
  /**
   * @brief Construct driver with optional SQLite-specific parameters.
   * @param parameters Map of parameters (see class description).
   */
  DBDriverSqlite3(const ParametersMap &parameters = ParametersMap());
  explicit DBDriverSqlite3(const DatabaseParams &parameters);
  virtual ~DBDriverSqlite3();

  /**
   * @brief True when the database runs in RAM instead of on disk.
   * @return True if the URL is empty or startup database.sqlite.in_memory is enabled.
   */
  virtual bool isInMemory() const { return getUrl().empty() || _dbInMemory; }

 protected:
  virtual bool connectDatabaseQuery(const std::string &url, bool overwritten = false, bool readOnly = false);
  virtual void disconnectDatabaseQuery(bool save = true, const std::string &outputUrl = "");
  virtual bool isConnectedQuery() const;
  virtual unsigned long getMemoryUsedQuery() const;  // In bytes
  virtual long getNodesMemoryUsedQuery() const;
  virtual long getLinksMemoryUsedQuery() const;
  virtual long getImagesMemoryUsedQuery() const;
  virtual long getDepthImagesMemoryUsedQuery() const;
  virtual long getCalibrationsMemoryUsedQuery() const;
  virtual long getGridsMemoryUsedQuery() const;
  virtual long getUserDataMemoryUsedQuery() const;
  virtual long getFeaturesMemoryUsedQuery() const;
  virtual long getStatisticsMemoryUsedQuery() const;
  virtual int getLastNodesSizeQuery() const;
  virtual int getTotalNodesSizeQuery() const;
  virtual ParametersMap getLastParametersQuery() const;
  virtual std::map<std::string, float> getStatisticsQuery(int nodeId, double &stamp, std::vector<int> *wmState) const;
  virtual std::map<int, std::pair<std::map<std::string, float>, double> > getAllStatisticsQuery() const;
  virtual std::map<int, std::vector<int> > getAllStatisticsWmStatesQuery() const;

  virtual void executeNoResultQuery(const std::string &sql) const;

  virtual void getWeightQuery(int nodeId, int &weight) const;

  virtual void saveQuery(const std::list<Node *> &nodes, const std::map<int, SensorRecord> &records);
  virtual void updateQuery(const std::list<Node *> &nodes) const;
  void saveWorkingMemoryQuery(const std::vector<Node *> &nodes, const RunInfo &info) const override;
  void addInfoAfterRunQuery(int size, const RunInfo &info) const override;

  virtual void addLinkQuery(const Link &link) const;
  virtual void updateLinkQuery(const Link &link) const;

  virtual void updateCalibrationQuery(int nodeId, const std::vector<camera::Perspective> &models) const;

  virtual void updateDepthImageQuery(int nodeId, const cv::Mat &image) const;

  virtual void addStatisticsQuery(const Statistics &statistics, bool saveWmState) const;
  virtual void savePreviewImageQuery(const cv::Mat &image) const;
  virtual cv::Mat loadPreviewImageQuery() const;
  std::map<int, RootFrame> load_root_frames_query() const override;
  RootFrame create_root_frame_query(int anchor, const StateGroupf &pose) override;
  virtual void saveOptimizedPosesQuery(const std::map<int, StateGroupf> &optimizedPoses, const StateGroupf &lastPose) const;
  virtual std::map<int, StateGroupf> loadOptimizedPosesQuery(StateGroupf *lastPose = 0) const;
  void save_trajectory_query(const std::vector<OdomObservation> &) const override;
  std::vector<OdomObservation> load_recent_trajectory_query(int limit) const override;
  int trajectory_high_water_query() const override;
  void save_localmap_query(int anchor_id, const cv::Mat &archive) const override;
  void save_visit_history_query(const std::string &archive) const override;
  std::string load_visit_history_query() const override;
  std::set<int> submap_neighbors_query(int id) const override;
  void merge_submap_query(int retained, int removed, const std::vector<Node *> &nodes, const cv::Mat &archive,
                          const std::map<int, StateGroupf> &poses, const StateGroupf &last_pose, const StateGroupf &retained_from_removed) override;
  cv::Mat load_submap_bundle_query(int anchor) const override;
  std::map<int, int> submap_members_query() const override;
  void refresh_submap_query(Node &target, int source, const cv::Mat &bundle) override;
  void consolidate_submap_query(Node &, int, const cv::Mat &, const cv::Mat &, const cv::Mat &) override;
  std::map<int, cv::Mat> load_localmaps_query() const override;
  std::vector<int> localmap_ids_query() const override;
  void visit_retrieval_summaries_query(const RetrievalVisitor &visitor) const override;
  void visit_global_descriptors_query(const RetrievalVisitor &visitor) const override;
  cv::Mat load_localmap_query(int anchor_id) const override;
  virtual void save2DMapQuery(const cv::Mat &map, float xMin, float yMin, float cellSize) const;
  virtual cv::Mat load2DMapQuery(float &xMin, float &yMin, float &cellSize) const;

  // Load objects
  virtual void loadLastNodesQuery(std::list<Node *> &nodes, bool loadWordIdsOnly) const;
  virtual void loadNodesQuery(const std::list<int> &ids, std::list<Node *> &nodes, bool loadWordIdsOnly) const;
  virtual void loadLinksQuery(int nodeId, std::multimap<int, Link> &links, std::optional<Link::Type> type = std::nullopt) const;

  virtual void loadNodeDataQuery(std::list<SensorRecord *> &records, bool images = true, bool userData = true, bool occupancyGrid = true) const;
  virtual bool getCalibrationQuery(int nodeId, std::vector<camera::Perspective> &models) const;
  virtual bool getNodeInfoQuery(int nodeId, StateGroupf &pose, int &mapId, int &weight, std::string &label, double &stamp,
                                std::vector<float> &velocity, EnvSensors &sensors) const;
  std::shared_ptr<const bow::FeatureBlock> load_loop_features(int node_id) const override;
  std::shared_ptr<const featmap::LandmarkMap> load_map_features(int node_id) const override;
  void write_features(const std::list<Node *> &nodes, bool update, bool store_history = true) const;
  virtual void getLocalFeaturesQuery(int nodeId, std::multimap<int, int> &words, std::vector<cv::KeyPoint> &keypoints,
                                     std::vector<cv::Point3f> &points, cv::Mat &descriptors) const;
  virtual void getLastNodeIdsQuery(std::set<int> &ids) const;
  virtual void getAllNodeIdsQuery(std::set<int> &ids, bool ignoreChildren, bool ignoreBadNodes, bool ignoreIntermediateNodes) const;
  virtual void getAllOdomPosesQuery(std::map<int, StateGroupf> &poses, bool ignoreChildren, bool ignoreIntermediateNodes) const;
  virtual void getAllLinksQuery(std::multimap<int, Link> &links, bool ignoreNullLinks) const;
  virtual void getLastIdQuery(const std::string &tableName, int &id, const std::string &fieldName = "id") const;
  virtual void getInvertedIndexNiQuery(int nodeId, int &ni) const;
  virtual void getNodeIdByLabelQuery(const std::string &label, int &id) const;
  virtual void getAllLabelsQuery(std::map<int, std::string> &labels) const;

 private:
  static std::optional<Link> read_link(sqlite3_stmt *, bool ignoreNullLinks = false);
  std::string queryStepNode() const;
  std::string queryStepCalibrationUpdate() const;
  std::string queryStepDepthUpdate() const;
  std::string queryStepSensorData() const;
  std::string queryStepLinkUpdate() const;
  std::string queryStepLink() const;
  std::string queryStepGlobalDescriptor() const;
  void stepNode(sqlite3_stmt *ppStmt, const Node *s, const EnvSensors &sensors) const;
  void stepCalibrationUpdate(sqlite3_stmt *ppStmt, int nodeId, const std::vector<camera::Perspective> &models) const;
  void stepDepthUpdate(sqlite3_stmt *ppStmt, int nodeId, const cv::Mat &image) const;
  void stepSensorData(sqlite3_stmt *ppStmt, const SensorRecord &record) const;
  void stepLink(sqlite3_stmt *ppStmt, const Link &link) const;
  void stepGlobalDescriptor(sqlite3_stmt *ppStmt, int nodeId, const GlobalDescriptor &descriptor) const;

 private:
  void loadWordsQuery(std::list<Node *> &nodes) const;
  void loadWordIdsQuery(std::list<Node *> &nodes) const;
  void loadLinksQuery(std::list<Node *> &nodes) const;
  int loadOrSaveDb(sqlite3 *pInMemory, const std::string &fileName, int isSave) const;

 protected:
  sqlite3 *_ppDb;        ///< Open SQLite connection (null when disconnected)
  std::string _version;  ///< Schema version read from the database

 private:
  std::atomic<unsigned long> _memoryUsedEstimate;
  std::atomic<bool> _dbInMemory;
  unsigned int _cacheSize;
  int _journalMode;
  int _synchronous;
  int _tempStore;
};

}  // namespace sapphire::backend

#endif /* DBDRIVERSQLITE3_H_ */
