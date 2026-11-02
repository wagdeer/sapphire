#include "module/mapping.hpp"
#include "module/relocalizer.hpp"
#include "database/memory.hpp"
#include "tools/logging.hpp"
#include <cassert>
#include <iostream>
using namespace sapphire;
using sapphire::GaussianCloudPtr;
GaussianCloudPtr cloud() {
  auto c = std::make_shared<GaussianCloud>();
  for (int i = 0; i < 600; ++i) {
    sapphire::GaussianPoint p;
    p.mean = Eigen::Vector3f(3 * std::sin(i * 1.37), 2 * std::cos(i * .73), 1.5 * std::sin(i * .39) + .2 * std::cos(i * .12));
    p.covariance = Eigen::Vector3f(.008f, .02f, .04f).asDiagonal();
    p.N = 17;
    p.radius = .6f;
    p.voxel_key = {i, -i, i + 2, i % 3};
    p.is_plane = i % 2;
    c->push_back(p);
  }
  return c;
}
sapphire::SE3f truth() {
  return sapphire::SE3f::fromRotationTranslation(
      (Eigen::AngleAxisf(.19f, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-.11f, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(.07f, Eigen::Vector3f::UnitX()))
          .toRotationMatrix(),
      Eigen::Vector3f(.45f, -.27f, .14f));
}
GaussianCloudPtr transformedQuery() {
  auto out = std::make_shared<GaussianCloud>();
  auto t = truth().inverse();
  const auto source = cloud();
  for (const auto& p : *source) out->push_back(p.transformed(t.rotationMatrix().cast<double>(), t.translationVector().cast<double>()));
  return out;
}
sapphire::SE3f pose(float x, float y = 0, float z = 0) { return sapphire::SE3f::fromTranslation(Eigen::Vector3f(x, y, z)); }

MappingParameters settings() {
  MappingParameters p;
  p.topograph.persistent_root_frames=true;
  p.topograph.reduce_graph=false;
  p.relocalizer.spatial_enabled=true;
  p.relocalizer.spatial_max_graph_hops=0;
  p.memory.recent_nodes=1;
  p.registration.lidar_bbs_enabled=false;
  return p;
}
void inspect(Mapping& m, bool joined) {
  std::map<int,SE3f> poses;std::multimap<int,Link> links;m.getCompleteGraph(poses,links);
  assert(poses.size()==6);
  for(int id=1;id<=6;++id) {
    auto n=m.getNodeCopy(id);
    assert(n.mapId()==(id+1)/2);
    assert((n.getPose().matrix()-pose(id%2?0:.1).matrix()).norm()<1e-6);
    const auto expected=(id==3||id==4)?transformedQuery():cloud();
    assert(n.lidar_cloud()->size()==expected->size());
    for(size_t i=0;i<expected->size();++i) {
      assert(n.lidar_cloud()->at(i).mean==expected->at(i).mean);
      assert(n.lidar_cloud()->at(i).covariance==expected->at(i).covariance);
    }
  }
  assert((poses.at(1).matrix()-SE3f::identity().matrix()).norm()<1e-5);
  assert((poses.at(5).matrix()-SE3f::identity().matrix()).norm()<1e-5);
  const auto expected=joined?truth():SE3f::identity();
  assert((poses.at(3).matrix()-expected.matrix()).norm()<.01);
  assert((poses.at(4).matrix()-expected.compose(pose(.1)).matrix()).norm()<.01);
  assert(m.getMemory()->metric_component(1).size()==(joined?4:2));
  assert(m.getMemory()->metric_component(5).size()==2);
  int cross=0;
  for(const auto& [_,edge]:links) if(edge.isRelocalization()) {
    ++cross;
    assert((edge.transform().matrix()-(edge.from()==1?truth():truth().inverse()).matrix()).norm()<.01);
  }
  assert(joined?cross>=1:cross==0);
}
int main(int argc,char**argv) try {
  tools::configure({});assert(argc==3);
  Mapping m;m.init(settings(),argv[2]);
  const std::string mode=argv[1];
  if(mode=="write") {
    for(int segment=0;segment<3;++segment) {
      if(segment)m.triggerNewMap();
      for(int j=0;j<2;++j) {
        auto g=segment==1?transformedQuery():cloud();
        assert(m.process_node(std::make_unique<Node>(g,10*segment+j,pose(.1*j)),Mat66_t<double>::Identity()));
      }
    }
    m.checkpoint();inspect(m,false);
    bool fenced=false;
    try {m.join_segments(1,3,SE3f::identity());} catch(const std::logic_error&) {fenced=true;}
    assert(fenced);
  } else if(mode=="verify" || mode=="continue") {
    inspect(m,true);
    if(mode=="continue") {
      m.triggerNewMap();
      assert(m.process_node(std::make_unique<Node>(cloud(),100,SE3f::identity()),Mat66_t<double>::Identity()));
      assert(m.getLastLocationId()==7 && m.getMemory()->metric_component(1).size()==4);
    }
  } else if(mode=="join") {
    assert(m.join_segments(3,1,SE3f::identity()));
    inspect(m,true);
    assert(m.getWorldOptimizedPoses().size()==4);
    const auto aliases=m.getRootFrameAliases();
    assert(aliases.size()==1 && (aliases.begin()->second.matrix()-truth().matrix()).norm()<.01);
  } else if(mode=="reject") {
    assert(!m.join_segments(1,3,pose(100)));
    inspect(m,false);
  } else throw std::invalid_argument("unknown test mode");
  m.close();std::cout<<"PASS segment graph and frozen evidence\n";
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
