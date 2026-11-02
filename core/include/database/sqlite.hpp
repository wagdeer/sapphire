#ifndef DBDRIVERSQLITE3_H_
#define DBDRIVERSQLITE3_H_
#include <common/common_type.hpp>
#include <parameters.hpp>

#include "common/se3_state.hpp"
#include "database/driver.hpp"

typedef struct sqlite3_stmt sqlite3_stmt;
typedef struct sqlite3 sqlite3;

namespace sapphire {

class DBDriverSqlite3 : public DBDriver {
 public:
  DBDriverSqlite3(const ParametersMap &parameters = ParametersMap());
  explicit DBDriverSqlite3(const DatabaseParams &parameters);
  virtual ~DBDriverSqlite3();

  virtual bool isInMemory() const { return getUrl().empty() || _dbInMemory; }

 protected:
  virtual bool connectDatabaseQuery(const std::string &url, bool overwritten = false, bool readOnly = false);
  virtual void disconnectDatabaseQuery(bool save = true, const std::string &outputUrl = "");
  virtual bool isConnectedQuery() const;
  virtual unsigned long getMemoryUsedQuery() const;  // In bytes
  virtual long getNodesMemoryUsedQuery() const;
  virtual long getLinksMemoryUsedQuery() const;


  virtual long getStatisticsMemoryUsedQuery() const;
  virtual int getLastNodesSizeQuery() const;
  virtual int getTotalNodesSizeQuery() const;
  virtual ParametersMap getLastParametersQuery() const;
  virtual std::map<std::string, float> getStatisticsQuery(int nodeId, double &stamp, std::vector<int> *wmState) const;
  virtual std::map<int, std::pair<std::map<std::string, float>, double> > getAllStatisticsQuery() const;
  virtual std::map<int, std::vector<int> > getAllStatisticsWmStatesQuery() const;

  virtual void executeNoResultQuery(const std::string &sql) const;

  virtual void getWeightQuery(int nodeId, int &weight) const;

  virtual void saveQuery(const std::list<Node *> &nodes);
  virtual void updateQuery(const std::list<Node *> &nodes) const;
  void saveWorkingMemoryQuery(const std::vector<Node *> &nodes, const RunInfo &info) const override;
  void addInfoAfterRunQuery(int size, const RunInfo &info) const override;

  virtual void addLinkQuery(const Link &link) const;
  virtual void updateLinkQuery(const Link &link) const;

  virtual void addStatisticsQuery(const Statistics &statistics, bool saveWmState) const;

  std::map<int, RootFrame> load_root_frames_query() const override;
  RootFrame create_root_frame_query(int anchor, const SE3f &pose) override;
  virtual void saveOptimizedPosesQuery(const std::map<int, SE3f> &optimizedPoses, const SE3f &lastPose) const;
  virtual std::map<int, SE3f> loadOptimizedPosesQuery(SE3f *lastPose = 0) const;
  void save_trajectory_query(const std::vector<OdomObservation> &) const override;
  std::vector<OdomObservation> load_recent_trajectory_query(int limit) const override;
  int trajectory_high_water_query() const override;
  virtual void loadLastNodesQuery(std::list<Node *> &nodes) const;
  virtual void loadNodesQuery(const std::list<int> &ids, std::list<Node *> &nodes) const;
  std::optional<GaussianCloudSize> node_geometry_size_query(int id) const override;
  virtual void loadLinksQuery(int nodeId, std::multimap<int, Link> &links, std::optional<Link::Type> type = std::nullopt) const;


  virtual bool getNodeInfoQuery(int nodeId, SE3f &pose, int &mapId, int &weight, std::string &label, double &stamp,
                                std::vector<float> &velocity) const;

  virtual void getLastNodeIdsQuery(std::set<int> &ids) const;
  virtual void getAllNodeIdsQuery(std::set<int> &ids, bool ignoreChildren, bool ignoreBadNodes, bool ignoreIntermediateNodes) const;
  virtual void getAllOdomPosesQuery(std::map<int, SE3f> &poses, bool ignoreChildren, bool ignoreIntermediateNodes) const;
  virtual void getAllLinksQuery(std::multimap<int, Link> &links, bool ignoreNullLinks) const;
  virtual void getLastIdQuery(const std::string &tableName, int &id, const std::string &fieldName = "id") const;

  virtual void getNodeIdByLabelQuery(const std::string &label, int &id) const;
  virtual void getAllLabelsQuery(std::map<int, std::string> &labels) const;

 private:
  static std::optional<Link> read_link(sqlite3_stmt *, bool ignoreNullLinks = false);
  std::string queryStepNode() const;

  std::string queryStepLinkUpdate() const;
  std::string queryStepLink() const;

  void stepNode(sqlite3_stmt *ppStmt, const Node *s) const;

  void stepLink(sqlite3_stmt *ppStmt, const Link &link) const;

 private:
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

}  // namespace sapphire

#endif
