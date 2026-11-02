#include "database/memory.hpp"
#include "database/sqlite.hpp"
#include "module/relocalizer.hpp"
#include "module/mapping.hpp"
#include "solver/relocalization.hpp"
#include "tools/logging.hpp"
#include <cassert>
#include <iostream>
#include <omp.h>
#include <opencv2/core.hpp>
#include <random>
using namespace sapphire;
SE3f tilt() {
  return SE3f::fromRotationTranslation(
      (Eigen::AngleAxisf(-.17f, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(.13f, Eigen::Vector3f::UnitX()))
          .toRotationMatrix(),
      Eigen::Vector3f::Zero());
}
SE3f truth() {
  return SE3f::fromRotationTranslation(
      Eigen::AngleAxisf(1.2f, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
          tilt().rotationMatrix(),
      {4.3f, -3.7f, .8f});
}
GaussianCloudPtr cloud(bool negative = false) {
  auto out = std::make_shared<GaussianCloud>();
  std::mt19937 engine(19);
  std::uniform_real_distribution<float> uniform(-1, 1);
  for (int i = 0; i < 1000; ++i) {
    GaussianPoint p;
    const int group = i % 3;
    p.mean = Eigen::Vector3f(uniform(engine) * (group == 0 ? 7 : 2) + group * 3,
                             uniform(engine) * (group == 1 ? 6 : 1) - group * 2,
                             uniform(engine) * (group == 2 ? 3 : 1) + group);
    if (negative)
      p.mean *= 30; // incompatible scale/shape, not merely another origin
    p.covariance = Eigen::Vector3f(.008f, .02f, .04f).asDiagonal();
    p.N = 17;
    p.radius = .6f;
    p.voxel_key = {i, -i, i + 2, i % 3};
    p.is_plane = true;
    out->push_back(p);
  }
  return out;
}
GaussianCloudPtr query() {
  auto out = std::make_shared<GaussianCloud>();
  const auto t = truth().inverse();
  const auto source = cloud();
  for (const auto &p : *source)
    out->push_back(p.transformed(t.rotationMatrix().cast<double>(),
                                 t.translationVector().cast<double>()));
  return out;
}
MappingParameters config(bool enabled = true) {
  MappingParameters p;
  p.topograph.persistent_root_frames = true;
  p.topograph.reduce_graph = false;
  p.memory.recent_nodes = 1;
  p.relocalizer.global_enabled = enabled;
  p.relocalizer.spatial_enabled = true;
  p.relocalizer.spatial_max_graph_hops = 0;
  p.relocalizer.path_max_nodes = 1;
  p.relocalizer.spatial_radius_m = 20;
  p.registration.lidar_bbs_enabled = false;
  return p;
}
void check(Mapping &m, int count) {
  std::map<int, SE3f> poses;
  std::multimap<int, Link> links;
  m.getCompleteGraph(poses, links);
  assert(poses.size() == size_t(count));
  assert((poses.at(1).matrix() - SE3f::identity().matrix()).norm() < 1e-5);
  assert((poses.at(2).matrix() - truth().matrix()).norm() < .02);
  assert(m.getMemory()->metric_component(4) == std::set<int>{4});
  assert((poses.at(4).matrix() - SE3f::identity().matrix()).norm() < 1e-5);
  auto expected = std::set<int>{1, 2, 3};
  if (count == 6) {
    expected.insert(5);
    expected.insert(6);
  };
  assert(m.getMemory()->metric_component(1) == expected);
  bool edge12 = false;
  for (const auto &[_, e] : links)
    if (e.isRelocalization() && (e.from() == 1 || e.to() == 1)) {
      const auto expected_pose=[](int id) {return id==2 || id==3 ? truth() : SE3f::identity();};
      const auto t = expected_pose(e.from()).inverse().compose(expected_pose(e.to()));
      assert((e.transform().matrix() - t.matrix()).norm() < .02);
      edge12 = true;
    }
  assert(edge12);
  assert(m.getNodeCopy(2).getPose().matrix() == tilt().matrix());
  const auto frozen = query();
  const auto saved = m.getNodeCopy(2).lidar_cloud();
  assert(saved->size() == frozen->size());
  for (size_t i = 0; i < saved->size(); ++i) {
    assert(saved->at(i).mean == frozen->at(i).mean);
    assert(saved->at(i).covariance == frozen->at(i).covariance);
  }
}
void ambiguity() {
  RegistrationParams p;
  Relocalizer::Verification a,b;
  a.component=1;a.datum_from_anchor=truth();a.alignment.T_target_query=truth().inverse();
  a.alignment.fitness=.1;a.alternatives={{a.alignment.T_target_query,.1}};
  b=a;b.datum_from_anchor=tilt();b.alignment.T_target_query=tilt().inverse();b.alternatives={{b.alignment.T_target_query,.11}};
  std::string reason;
  assert(Relocalizer::select({a,b},p,reason)); // different anchors, same physical body pose
  b.alignment.fitness=.49;b.alternatives={{b.alignment.T_target_query,.49}};
  b.component=2;
  assert(!Relocalizer::select({a,b},p,reason) && reason=="ambiguous_geometry");
  b.component=1;b.datum_from_anchor=SE3f::fromTranslation({100,0,0}).compose(b.datum_from_anchor);
  assert(!Relocalizer::select({a,b},p,reason)); // repeated road in one metric component
  const auto gauge=truth().compose(SE3f::fromTranslation({-900,200,37}));
  a.datum_from_anchor=gauge.compose(a.datum_from_anchor);b.datum_from_anchor=gauge.compose(b.datum_from_anchor);
  assert(!Relocalizer::select({b,a},p,reason)); // gauge and candidate order invariant
  b.alignment.T_target_query={}; // Failed precision gate, but supported alternative remains unresolved.
  assert(!Relocalizer::select({a,b},p,reason));
  b.alternatives.clear();
  assert(Relocalizer::select({b,a},p,reason));
  b.resource_failure="budget exceeded";
  assert(!Relocalizer::select({a,b},p,reason) && reason=="incomplete_geometry_search");
  b.resource_failure.clear();a.alternatives.push_back({SE3f::fromTranslation({50,0,0}),.1});
  assert(!Relocalizer::select({a},p,reason)); // one target, two initializations
  a.datum_from_anchor=SE3f::identity();a.alignment.T_target_query=SE3f::identity();
  a.alternatives={{SE3f::fromTranslation({-1.5f,0,0}),.11},{SE3f::fromTranslation({1.5f,0,0}),.12}};
  assert(!Relocalizer::select({a},p,reason)); // best middle pose cannot conceal disagreeing endpoints
}
void evidence(const std::string& file) {
  for (bool connected : {false,true}) for (bool same_session : {false,true}) {
    const auto path=file+std::to_string(connected)+std::to_string(same_session);
    const auto remote=SE3f::fromTranslation({900,-300,70});
    std::map<int,SE3f> poses{{1,SE3f::identity()},{2,SE3f::fromTranslation({30,0,0})},{3,SE3f::identity()},
                            {10,connected?truth():remote}};
    const int session=same_session?0:17;
    {
      DBDriverSqlite3 db; assert(db.openConnection(path));
      for (int id : {1,2,3,10}) {
        auto n=std::make_unique<Node>(id,id==1?0:session,0,id,"",poses.at(id));
        n->set_lidar_cloud(id==1?cloud():id==10?query():cloud(true));
        if(connected) for(const auto [from,to]:{std::pair{1,2},std::pair{2,10}}) {
          Link e(from,to,Link::kOdometry,poses.at(from).inverse().compose(poses.at(to)));
          if(id==from)n->addLink(e);if(id==to)n->addLink(e.inverse());
        }
        assert(db.asyncSave(std::move(n)));
      }
      db.closeConnection();
    }
    auto p=config(); Memory memory(p);assert(memory.init(path));
    std::unique_ptr<Node> q(memory.database()->loadNode(10));
    Relocalizer finder(memory,p.relocalizer,p.registration);
    const auto without=finder.candidates(*q,poses);
    assert(!without.empty() && without.front()==1);
    assert(finder.verify({3,{3}},10,poses).alignment.T_target_query.isNull());
    auto pass=finder.verify({1,{1}},10,poses);
    assert(!pass.alignment.T_target_query.isNull());
    assert((pass.alignment.T_target_query.matrix()-truth().matrix()).norm()<.02);
    assert(pass.initialization_trace.rfind(connected?"graph_pose_seed;":"automatic_pagor;",0)==0);
    if (!connected) {
      auto changed=poses;changed.at(10)=truth();
      assert(finder.candidates(*q,changed)==without); // independent origins cannot affect retrieval
      p.relocalizer.global_enabled=false;
      Relocalizer local_only(memory,p.relocalizer,p.registration);
      assert(local_only.candidates(*q,poses).empty());
      const auto unavailable=local_only.verify({1,{1}},10,poses);
      assert(unavailable.alignment.T_target_query.isNull() && !unavailable.alignment.failure.empty());
    } else {
      auto missing=poses;missing.at(10)={};
      const auto absent=finder.verify({1,{1}},10,missing);
      assert(!absent.alignment.T_target_query.isNull());
      assert(absent.initialization_trace.rfind("automatic_pagor;",0)==0);
      assert(finder.candidates(*q,missing).front()==1);
      p.relocalizer.spatial_enabled=false;
      const auto global=Relocalizer(memory,p.relocalizer,p.registration).verify({1,{1}},10,poses);
      assert(!global.alignment.T_target_query.isNull());
      assert(global.initialization_trace.rfind("automatic_pagor;",0)==0);
      assert((global.alignment.T_target_query.matrix()-truth().matrix()).norm()<.02);
    }
    memory.close(false);
  }
}
int main(int argc, char **argv) try {
  assert(argc == 3);
  tools::configure({});
  cv::setNumThreads(1);
  omp_set_num_threads(1);
  const std::string mode = argv[1];
  if(mode=="evidence") { ambiguity();evidence(argv[2]);std::cout<<"PASS session-independent pose evidence and unknown-datum geometry\n";return 0; }
  Mapping m;
  m.init(config(mode != "write"), argv[2]);
  const auto add = [&](GaussianCloudPtr c, SE3f pose) {
    assert(m.process_node(
        std::make_unique<Node>(c, m.getLastLocationId() + 1, pose),
        Mat66_t<double>::Identity()));
  };
  if(mode=="async") {
    m.enable_async_relocalization();
    add(cloud(),SE3f::identity());m.triggerNewMap();
    add(query(),tilt());assert(m.relocalization_pending());
    // Reset/admit another independent segment while the task owns only old clouds.
    m.triggerNewMap();add(cloud(true),SE3f::identity());
    m.poll_relocalization(true);assert(!m.relocalization_pending());
    assert(m.getMemory()->metric_component(1)==std::set<int>({1,2}));
    assert(m.getMemory()->metric_component(3)==std::set<int>{3});
    assert((m.getMapCorrection().matrix()-SE3f::identity().matrix()).norm()<1e-5);
    m.checkpoint();m.close();
    m.init(config(false),argv[2]);
    assert(m.getMemory()->metric_component(1)==std::set<int>({1,2}));
    assert(m.getMemory()->metric_component(3)==std::set<int>{3});
    m.close();return 0;
  }
  if (mode == "write" || mode == "live") {
    add(cloud(), SE3f::identity());
    m.triggerNewMap();
    add(query(), tilt());
    add(query(), tilt());
    m.triggerNewMap();
    add(cloud(true), SE3f::identity());
    m.checkpoint();
    if (mode == "live")
      check(m, 4);
    else
      assert(m.getMemory()->metric_component(1) == std::set<int>{1});
  } else if (mode == "auto") {
    assert(m.relocalize() >= 1);
    check(m, 4);
  } else if (mode == "verify")
    check(m, 4);
  else if (mode == "continue") {
    check(m, 4);
    m.triggerNewMap();
    add(cloud(), SE3f::identity());
    add(cloud(), SE3f::identity());
    m.checkpoint();
    check(m, 6);
    assert((m.getMapCorrection().matrix() - SE3f::identity().matrix()).norm() <
           .02);
    assert(m.getRootFrameAliases().size() == 2);
  } else if (mode == "verify4")
    check(m, 6);
  else if (mode == "same") {
    add(cloud(), SE3f::identity());
    add(cloud(true), SE3f::fromTranslation({30, 0, 0}));
    add(query(), truth());
    assert(m.getRelocalizationId() == 1);
    std::map<int, SE3f> poses;
    std::multimap<int, Link> links;
    m.getCompleteGraph(poses, links);
    bool found = false;
    for (const auto &[_, e] : links)
      if (e.isRelocalization()) {
        found = true;
        assert((e.transform().matrix() -
                (e.from() == 1 ? truth() : truth().inverse()).matrix())
                   .norm() < .02);
      }
    assert(found);
  } else
    throw std::invalid_argument("unknown mode");
  m.close();
  std::cout << "PASS automatic " << mode << '\n';
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n';
  return 1;
}
