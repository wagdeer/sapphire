#include "database/compression.hpp"
#include "database/memory.hpp"
#include "database/sqlite.hpp"
#include "module/relocalizer.hpp"
#include "module/mapping.hpp"
#include <fstream>
#include <chrono>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <unistd.h>
using namespace sapphire;
void require(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
GaussianCloudPtr cloud() {
  auto points = std::make_shared<GaussianCloud>();
  for (int i=0;i<600;++i) {
    GaussianPoint p;
    p.mean={float(3*std::sin(i*1.37)),float(2*std::cos(i*.73)),float(1.5*std::sin(i*.39))};
    p.covariance=Eigen::Vector3f(.008f,.02f,.04f).asDiagonal();p.N=17;p.radius=.6f;p.voxel_key={i,-i,i+2,0};
    points->push_back(p);
  }
  return points;
}
int main() {
  const auto file=(std::filesystem::temp_directory_path()/("sapphire-path-budget-"+std::to_string(getpid())+"-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".db")).string();
  try {
    const auto points=cloud();const auto bytes=points->size()*sizeof(GaussianPoint);
    const auto pose=SE3f::fromRotationTranslation(
        (Eigen::AngleAxisf(.19f,Eigen::Vector3f::UnitZ())*Eigen::AngleAxisf(-.11f,Eigen::Vector3f::UnitY())).toRotationMatrix(),{.45f,-.27f,.14f});
    auto query=std::make_shared<GaussianCloud>();
    for(const auto& p:*points)query->push_back(p.transformed(pose.rotationMatrix().transpose().cast<double>(),
        -(pose.rotationMatrix().transpose()*pose.translationVector()).cast<double>()));
    const std::size_t oversized_count=(64U*1024U*1024U)/sizeof(GaussianPoint)+1;
    {
      DBDriverSqlite3 db;require(db.openConnection(file),"create fixture");
      for(int id:{1,2,3,9}) {
        auto node=std::make_unique<Node>(id,0,0,id,"",id==9?pose:SE3f::identity());
        node->set_lidar_cloud(id==9?query:points);
        for (const auto [from,to] : {std::pair{1,2},std::pair{2,3},std::pair{3,9}}) {
          Link edge(from,to,Link::kOdometry,to==9?pose:SE3f::identity());
          if(id==from)node->addLink(edge);if(id==to)node->addLink(edge.inverse());
        }
        require(db.asyncSave(std::move(node)),"save fixture");
      }
      auto huge=std::make_shared<GaussianCloud>(oversized_count,points->front());
      auto node=std::make_unique<Node>(4,0,0,4,"",SE3f::identity());node->set_lidar_cloud(huge);
      require(db.asyncSave(std::move(node)),"save real >64 MiB fixture");
      require(db.node_geometry_size(4).value().bytes()==oversized_count*sizeof(GaussianPoint),"pending or committed size visible");
      db.emptyTrashes(false);
      require(db.node_geometry_size(1).value().bytes()==bytes && db.node_geometry_size(4).value().bytes()==huge->size()*sizeof(GaussianPoint),"cold archive trailer agrees with actual count");
      require(!db.node_geometry_size(99999),"missing row distinct from empty geometry");
      auto changed=std::make_unique<Node>(8,0,0,8,"",SE3f::identity());changed->set_lidar_cloud(points);
      require(db.asyncSave(std::move(changed)),"save mutable fixture");db.emptyTrashes(false);
      changed.reset(db.loadNode(8));changed->set_lidar_cloud({});
      require(db.asyncSave(std::move(changed)),"queue empty geometry replacement");
      require(db.node_geometry_size(8).value().bytes()==0,"latest queued NULL geometry masks old archive");
      db.emptyTrashes(false);require(db.node_geometry_size(8).value().bytes()==0,"committed NULL geometry size");
      // Prove metadata inspection and discarded supports don't decode a payload.
      db.executeNoResult("UPDATE Node SET lidar_geometry=CAST(x'00' || substr(lidar_geometry,2) AS BLOB) WHERE id=4");
      require(db.node_geometry_size(4).value().bytes()==oversized_count*sizeof(GaussianPoint),"preflight reads only shape, not compressed payload");
      db.closeConnection();
    }
    MappingParameters p;p.registration.lidar_bbs_enabled=false;
    p.relocalizer.path_max_nodes=3;
    Memory memory(p);require(memory.init(file),"open cold geometry");
    const std::map<int,SE3f> poses{{1,SE3f::identity()},{2,SE3f::identity()},{3,SE3f::identity()},{4,SE3f::identity()},{9,pose}};
    require(!memory.getNode(4) && !memory.getNode(2),"fixture starts cold");
    Relocalizer finder(memory,p.relocalizer,p.registration);
    auto rejected=finder.verify({4,{4}},9,poses);
    require(!rejected.resource_failure.empty() && rejected.alignment.failure.empty() && rejected.alignment.gicp_ms==0,"oversize anchor is resource failure before GICP");
    require(!memory.getNode(4) && !memory.getNode(9),"oversize anchor does not activate either cloud");
    auto trimmed=finder.verify({2,{2,3,4}},9,poses);
    require(trimmed.path.nodes==std::vector<int>({2,3}) && trimmed.resource_failure.empty(),"remove oversized far endpoint, retain contiguous complete nodes");
    require(!trimmed.alignment.T_target_query.isNull() && trimmed.target_points==2*points->size(),"shortened path still verifies");
    require(trimmed.alignment.T_target_query.translationDistance(pose)<.001f,"asymmetric anchor-from-query convention");
    require(!memory.getNode(4),"omitted cold support never activated");
    auto two=p.relocalizer;two.path_max_bytes=2*bytes;
    auto result=Relocalizer(memory,two,p.registration).verify({2,{1,2,3}},9,poses);
    require(result.path.nodes==std::vector<int>({1,2}) && result.target_bytes==2*bytes,"3->2 at inclusive byte boundary");
    auto one=p.relocalizer;one.path_max_bytes=bytes;
    result=Relocalizer(memory,one,p.registration).verify({2,{1,2,3}},9,poses);
    require(result.path.nodes==std::vector<int>({2}) && result.target_points==points->size() && !result.alignment.T_target_query.isNull(),"3->1 preserves entire anchor and match");
    require(result.requested_bytes==3*bytes && result.target_bytes==bytes,"requested and effective payload observable");
    auto duplicate=Relocalizer(memory,one,p.registration).verify({2,{1,2,3}},9,poses,{{2}});
    require(duplicate.duplicate && duplicate.alignment.gicp_ms==0 && duplicate.target_points==0,"deduplicate effective path before matching");
    auto another=Relocalizer(memory,one,p.registration).verify({1,{1,2,3}},9,poses,{{2}});
    require(!another.duplicate && another.path.nodes==std::vector<int>({1}) && !another.alignment.T_target_query.isNull(),"same requested membership can fit a different anchor; dedup only effective geometry");
    auto low=one;--low.path_max_bytes;
    result=Relocalizer(memory,low,p.registration).verify({2,{1,2,3}},9,poses);
    require(!result.resource_failure.empty() && result.path.nodes==std::vector<int>({2}),"single Node is indivisible below boundary");
    bool corrupt=false;try{std::unique_ptr<Node> node(memory.database()->loadNode(4));}catch(const std::runtime_error&){corrupt=true;}
    require(corrupt,"full decode still rejects corrupt archive; preflight doesn't certify it");
    memory.close(false);
    // Inspect other candidates, but an untested resource-limited alternative
    // cannot be used as negative evidence to assert an unambiguous query.
    const auto online=std::filesystem::path(file+".online");std::filesystem::create_directories(online);
    MappingParameters config;config.registration.lidar_bbs_enabled=false;
    config.relocalizer.path_max_nodes=1;config.relocalizer.path_max_bytes=2*bytes;
    config.relocalizer.spatial_radius_m=2;config.relocalizer.spatial_max_graph_hops=0;
    config.statistics.log_enabled=true;
    Mapping mapping;mapping.init(config,(online/"map.db").string(),online.string());
    auto larger=std::make_shared<GaussianCloud>();
    for(int copy=0;copy<4;++copy)larger->insert(larger->end(),points->begin(),points->end());
    for(int i=1;i<=13;++i) {
      const auto at=SE3f::fromTranslation({i==1?0.0f:i==2?.1f:float(i)*5,0,0});
      require(mapping.process_node(std::make_unique<Node>(i==1?larger:points,i,at),Mat66_t<double>::Identity()),"online fixture admitted");
    }
    require(mapping.process_node(std::make_unique<Node>(points,14,SE3f::fromTranslation({.01f,0,0})),Mat66_t<double>::Identity()),"online closing node admitted");
    require(mapping.getRelocalizationId()==0,"incomplete query withholds commitment despite another geometric fit");
    mapping.close();
    std::ifstream trace(online/"relocalization.tsv");std::string line;bool limited=false,accepted=false;
    std::getline(trace,line);
    std::vector<std::string> columns;std::stringstream header(line);std::string field;
    while(std::getline(header,field,'\t'))columns.push_back(field);
    const auto resource=std::find(columns.begin(),columns.end(),"resource_limited");
    require(resource!=columns.end(),"resource diagnostic column exists");
    const auto index=std::size_t(resource-columns.begin());
    while(std::getline(trace,line)) {
      std::vector<std::string> values;std::stringstream row(line);
      while(std::getline(row,field,'\t'))values.push_back(field);
      require(values.size()>index,"resource diagnostic is present");
      if(line.rfind("14\t1\t",0)==0)limited=line.find("Anchor Gaussian payload")!=std::string::npos && values[index]=="1";
      if(line.rfind("14\t2\t",0)==0)accepted=values[index]=="0";
    }
    require(limited && accepted,"resource and geometric outcomes independently observable");
    std::filesystem::remove_all(online);
    std::filesystem::remove(file);
    std::cout<<"PASS path budget, real >64 MiB preflight, intact Node fallback, effective dedup\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';std::filesystem::remove(file);return 1;}
}
