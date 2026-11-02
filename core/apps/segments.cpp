// Optional archive maintenance. Never participates in the sensor input path.
#include "database/driver.hpp"
#include "module/mapping.hpp"
#include "parameters.hpp"
#include <opencv2/core.hpp>
#include <omp.h>
#include <sqlite3.h>
#include <unistd.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <memory>
#include <set>
using namespace sapphire;
namespace fs=std::filesystem;
namespace {
void copy_archive(const fs::path& input,const fs::path& output) {
  sqlite3 *source=nullptr,*destination=nullptr;
  const auto close=[](sqlite3* db){if(db)sqlite3_close(db);};
  const int opened=sqlite3_open_v2(input.c_str(),&source,SQLITE_OPEN_READONLY,nullptr);
  std::unique_ptr<sqlite3,decltype(close)> a(source,close);
  if(opened!=SQLITE_OK)throw std::runtime_error("Cannot read source archive");
  const int created=sqlite3_open_v2(output.c_str(),&destination,SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE,nullptr);
  std::unique_ptr<sqlite3,decltype(close)> b(destination,close);
  if(created!=SQLITE_OK)throw std::runtime_error("Cannot create output archive");
  sqlite3_backup* backup=sqlite3_backup_init(destination,"main",source,"main");
  if(!backup)throw std::runtime_error(sqlite3_errmsg(destination));
  const int copied=sqlite3_backup_step(backup,-1),finished=sqlite3_backup_finish(backup);
  if(copied!=SQLITE_DONE || finished!=SQLITE_OK)throw std::runtime_error("Archive backup failed; stop/checkpoint the source before joining");
}
void list(const fs::path& path) {
  std::unique_ptr<DBDriver> db(DBDriver::create());
  if(!db->openConnection(path.string(),false,true))throw std::runtime_error("Cannot open archive");
  const auto roots=db->load_root_frames();std::set<int> ids;db->getAllNodeIds(ids,false,false,false);
  std::multimap<int,Link> links;db->getAllLinks(links,true);
  std::map<int,std::set<int>> adjacent;
  for(const auto& [_,edge]:links)if(edge.from()>0 && edge.to()>0 && (edge.isOdometry()||edge.isRelocalization())) {
    adjacent[edge.from()].insert(edge.to());adjacent[edge.to()].insert(edge.from());
  }
  std::map<int,std::string> component;std::set<int> visited;
  for(int seed:ids)if(!visited.count(seed)) {
    std::vector<int> pending{seed};const RootFrame* oldest=nullptr;visited.insert(seed);
    for(size_t i=0;i<pending.size();++i) {
      const int id=pending[i];const auto root=roots.find(id);
      if(root!=roots.end() && (!oldest||root->second.creation_order<oldest->creation_order))oldest=&root->second;
      for(int peer:adjacent[id])if(ids.count(peer)&&visited.insert(peer).second)pending.push_back(peer);
    }
    if(!oldest)throw std::runtime_error("Saved component lacks a coordinate datum");
    for(int id:pending)component[id]=oldest->id;
  }
  struct Summary {size_t count=0;int first=0,last=0;double start=0,end=0;std::string frame;};
  std::map<int,Summary> segments;
  for(int id:ids) {
    SE3f pose;int segment,weight;std::string label;double stamp;std::vector<float> velocity;
    if(!db->getNodeInfo(id,pose,segment,weight,label,stamp,velocity))throw std::runtime_error("Missing saved node metadata");
    auto& s=segments[segment];if(!s.count){s.first=id;s.start=stamp;s.frame=component.at(id);}
    ++s.count;s.last=id;s.end=stamp;
  }
  std::cout<<std::setprecision(17)<<"segment,nodes,first_node,last_node,first_node_stamp,last_node_stamp,component_frame\n";
  for(const auto& [id,s]:segments)std::cout<<id<<','<<s.count<<','<<s.first<<','<<s.last<<','<<s.start<<','<<s.end<<','<<s.frame<<'\n';
  db->closeConnection(false);
}
}
int main(int argc,char**argv) try {
  cv::setNumThreads(1);omp_set_num_threads(1);
  if(argc==3 && std::string(argv[1])=="list") {list(argv[2]);return 0;}
  const bool automatic=argc==5 && std::string(argv[1])=="relocalize";
  if(!automatic && (argc!=14 || std::string(argv[1])!="join")) {
    std::cerr<<"Usage:\n  sapphire_segments list CLOSED_MAP.db\n"
      "  sapphire_segments relocalize SOURCE.db NEW_OUTPUT.db CONFIG.toml\n"
      "  sapphire_segments join SOURCE.db NEW_OUTPUT.db CONFIG.toml REFERENCE_NODE QUERY_NODE tx ty tz qx qy qz qw (diagnostic only)\n"
      "Seed T_reference_query maps query-node coordinates into reference-node coordinates.\n"
      "The source is unchanged; output appears only after accepted geometric verification and durable close.\n";
    return 2;
  }
  const fs::path input=fs::absolute(argv[2]),output=fs::absolute(argv[3]);
  if(!fs::is_regular_file(input)||fs::exists(output))throw std::invalid_argument("Source must exist and output must be new");
  if(!fs::is_directory(output.parent_path()))throw std::invalid_argument("Output parent directory must exist");
  auto config=load_parameters(argv[4],SensorParameters{}).pose_graph.mapping;
  config.topograph.persistent_root_frames=true;config.topograph.reduce_graph=false;
  if(automatic){config.relocalizer.global_enabled=true;config.statistics.log_enabled=true;}
  else config.relocalizer.spatial_enabled=false;
  SE3f seed;int reference=0,query=0;
  if(!automatic) {
  reference=std::stoi(argv[5]);query=std::stoi(argv[6]);
  Eigen::Quaternionf q(std::stof(argv[13]),std::stof(argv[10]),std::stof(argv[11]),std::stof(argv[12]));
  if(!q.coeffs().allFinite()||std::abs(q.norm()-1)>1e-3)throw std::invalid_argument("Seed quaternion must be finite and unit length");
  Eigen::Matrix4f matrix=Eigen::Matrix4f::Identity();matrix.topLeftCorner<3,3>()=q.normalized().toRotationMatrix();
  matrix.topRightCorner<3,1>()=Eigen::Vector3f(std::stof(argv[7]),std::stof(argv[8]),std::stof(argv[9]));
  seed=SE3f(matrix);if(!seed.isValid())throw std::invalid_argument("Invalid seed pose");
  }
  const auto suffix=std::to_string(getpid())+"_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto temporary=output.parent_path()/(output.filename().string()+".partial_"+suffix);
  if(!fs::create_directory(temporary))throw std::runtime_error("Cannot reserve temporary output");
  struct Cleanup {fs::path path;~Cleanup(){std::error_code error;fs::remove_all(path,error);}} cleanup{temporary};
  const auto copy=temporary/"map.db";copy_archive(input,copy);
  Mapping mapping;
  const fs::path log_directory=output.string()+".diagnostics";
  if(automatic && !fs::create_directory(log_directory))throw std::runtime_error("Diagnostic output already exists");
  mapping.init(config,copy.string(),automatic?log_directory.string():std::string{});
  if(automatic) {
    const auto count=mapping.relocalize();mapping.close();fs::create_hard_link(copy,output);
    std::cout<<"Automatic relocalization committed "<<count<<" measurements in "<<output<<"; unmatched components retained.\n";
    return 0;
  }
  const auto edge=mapping.join_segments(reference,query,seed);
  if(!edge) {mapping.close();std::cerr<<"No verified overlap; segments retained separately in unchanged source.\n";return 3;}
  mapping.close();
  // Atomic no-replace publication in the same filesystem; never overwrite a
  // destination created by another process while verification was running.
  fs::create_hard_link(copy,output);
  std::cout<<"Joined "<<reference<<" -> "<<query<<" in "<<output<<"\nT_reference_query:\n"<<edge->transform().matrix()<<'\n';
  return 0;
} catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
