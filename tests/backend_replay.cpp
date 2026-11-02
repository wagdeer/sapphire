// Replay archived immutable batches through Mapping for regression checks.
// Repeated passes are the same recorded traversal, not independent acquisitions.
#include "module/mapping.hpp"
#include "database/sqlite.hpp"
#include "database/compression.hpp"
#include <sqlite3.h>
#include <cstring>
#include "maplayer/topograph/node.hpp"
#include "parameters.hpp"
#include <Eigen/Cholesky>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <sys/resource.h>
using namespace sapphire;
int main(int argc,char** argv) {
 try {
  if(argc!=5 && argc!=6)throw std::invalid_argument("backend_replay SOURCE_DB CONFIG SENSORS OUTPUT [PASSES]");
  const int passes=argc==6?std::stoi(argv[5]):2;
  if(passes<1)throw std::invalid_argument("Passes must be positive");
  const std::filesystem::path out=argv[4];
  if(std::filesystem::exists(out))throw std::invalid_argument("Output exists");
  std::filesystem::create_directories(out);
  const auto start=std::chrono::steady_clock::now();
  auto config=load_parameters(argv[2],load_sensor_parameters(argv[3]).lidars.front()).pose_graph.mapping;

  Mapping mapping;mapping.init(config,(out/"map.db").string(),out.string());
  size_t backend_nodes=0;
  {
   // Offline fixture reader: historical snapshots have extra retired tables.
   // Read only frozen sensor geometry, native anchors and odometry factors;
   // this does not claim those databases can resume with the current runtime.
   sqlite3* handle=nullptr;
   if(sqlite3_open_v2(argv[1],&handle,SQLITE_OPEN_READONLY,nullptr)!=SQLITE_OK)throw std::runtime_error("Open source");
   std::unique_ptr<sqlite3,decltype(&sqlite3_close)> db(handle,sqlite3_close);
   auto prepare=[&](const char* sql){sqlite3_stmt* q=nullptr;if(sqlite3_prepare_v2(db.get(),sql,-1,&q,nullptr)!=SQLITE_OK)throw std::runtime_error(sqlite3_errmsg(db.get()));return std::unique_ptr<sqlite3_stmt,decltype(&sqlite3_finalize)>(q,sqlite3_finalize);};
   auto pose=[&](sqlite3_stmt* q,int column){if(sqlite3_column_bytes(q,column)!=48)throw std::runtime_error("Invalid fixture pose");SE3f::Coefficients p;std::memcpy(p.data(),sqlite3_column_blob(q,column),48);SE3f t(p);if(!t.isValid())throw std::runtime_error("Invalid fixture transform");return t;};
   // Decode only the current fixture Node. A retained vector would keep all
   // original clouds alive and inflate the replay working set. Re-read the same
   // ordered source rows on every pass and release each fixture after processing.
   for(int pass=0;pass<passes;++pass) {
    auto nodes=prepare("SELECT id,stamp,pose,lidar_geometry FROM Node WHERE lidar_geometry IS NOT NULL ORDER BY id");
    int code;
    while((code=sqlite3_step(nodes.get()))==SQLITE_ROW){
     const int id=sqlite3_column_int(nodes.get(),0);
     auto cloud=lidar::decode(cv::Mat(1,sqlite3_column_bytes(nodes.get(),3),CV_8UC1,const_cast<void*>(sqlite3_column_blob(nodes.get(),3))));
     auto node=std::make_unique<Node>(id,0,0,sqlite3_column_double(nodes.get(),1),"",pose(nodes.get(),2));node->set_lidar_cloud(std::move(cloud));
     auto links=prepare("SELECT to_id,transform,information_matrix FROM Link WHERE from_id=?1 AND to_id<from_id AND type=0 ORDER BY to_id");
     sqlite3_bind_int(links.get(),1,id);int status;
     while((status=sqlite3_step(links.get()))==SQLITE_ROW){
      if(sqlite3_column_bytes(links.get(),2)!=288)throw std::runtime_error("Invalid fixture information matrix");
      MatRC_t<6,6,double,Eigen::RowMajor> information;std::memcpy(information.data(),sqlite3_column_blob(links.get(),2),288);
      node->addLink(Link(id,sqlite3_column_int(links.get(),0),Link::kOdometry,pose(links.get(),1),information));
     }
     if(status!=SQLITE_DONE)throw std::runtime_error(sqlite3_errmsg(db.get()));
     Mat66_t<double> covariance=Mat66_t<double>::Identity()*.01;
     for(const auto& [id,edge]:node->getLinks())if(edge.isOdometry() && id<node->id()) {
      const auto forward=edge.inverse();covariance=forward.infMatrix().llt().solve(Mat66_t<double>::Identity());break;
     }
     if(!mapping.process_node(std::make_unique<Node>(node->lidar_cloud(),node->getStamp()+300*pass,node->getPose()),covariance))throw std::runtime_error("Frozen observation rejected");
     ++backend_nodes;
     node.reset();
    }
    if(code!=SQLITE_DONE)throw std::runtime_error(sqlite3_errmsg(db.get()));
   }
  }
  mapping.close();
  rusage usage{};getrusage(RUSAGE_SELF,&usage);
  std::ofstream f(out/"run.json");f<<"{\"mode\":\"frozen-node traversal\",\"backend_nodes\":"<<backend_nodes<<",\"wall_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<",\"cpu_s\":"<<usage.ru_utime.tv_sec+usage.ru_utime.tv_usec/1e6+usage.ru_stime.tv_sec+usage.ru_stime.tv_usec/1e6<<",\"peak_rss_kb\":"<<usage.ru_maxrss<<"}\n";
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
