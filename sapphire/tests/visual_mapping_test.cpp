#include <unistd.h>

#include <iostream>
#include <atomic>
#include <random>

#include "backend/registration/loop_closure.hpp"
#include "backend/grid/flat_ground.hpp"
#include "backend/storage/memory.hpp"
#include "backend/visual/visual_loop.hpp"
#include "backend/graph/pose_graph.hpp"
#include "pipeline.hpp"
#include "backend/visual/solver/visual_pnp.hpp"
#include "backend/visual/feature/visual_tracker.hpp"
using namespace sapphire;
namespace {
std::atomic<int> reject_stage{0}, coarse_attempts{0}, refined_attempts{0}, pnp_attempts{0}, scored_seeds{0};
}
namespace sapphire::database_detail {
void writableStorageTestPoint(const char *name, void *context) {
  if (std::string(name) == "visual-after-pnp") {
    ++pnp_attempts;
    auto &result = *static_cast<VisualPnpResult *>(context);
    if (reject_stage == 3 && result.T_target_query) result.T_target_query->translation().x() += 20;
  } else if (std::string(name) == "visual-seed-geometry") {
    ++scored_seeds;
  } else if (std::string(name) == "b2-after-bbs") {
    const int attempt = coarse_attempts.fetch_add(1);
    if (reject_stage == 1 && attempt == 0) static_cast<BbsResult *>(context)->accepted = false;
  } else if (std::string(name) == "b2-after-gicp") {
    const int attempt = refined_attempts.fetch_add(1);
    if (reject_stage == 2 && attempt == 0) static_cast<GicpResult *>(context)->accepted = false;
  }
}
}  // namespace sapphire::database_detail
void check(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

VisualLoopParameters configuration(const std::string &mode) {
  VisualLoopParameters c;
  c.enabled = true;
  c.mode = mode;
  c.left.width = 640;
  c.left.height = 480;
  c.left.intrinsics = {300, 300, 320, 240};
  c.left.camera_to_imu_rotation = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  c.left.camera_to_imu_translation = {.2, -.1, .05};
  c.right = c.left;
  return c;
}
namespace sapphire {
struct SlamPipelineTestAccess {
  static void checkImageQueue(SlamPipeline &p, std::size_t count, std::size_t bytes) {
    std::lock_guard<std::mutex> lock(p.image_mutex_);
    check(p.image_buffer_.size() == count && p.image_buffer_bytes_ == bytes,
          "pending image count and pixel bytes obey both budgets");
    check(p.image_buffer_.front().gray.at<std::uint8_t>(0, 0) == 53,
          "queued pixels remain owned after original raw message is released");
  }
};
}
void imageQueueOwnershipTest(int side) {
  SapphireParameters p; p.general.save_map = 0; p.navi_map.enabled = false;
  p.pose_graph.database_path = "/tmp/sapphire-image-owner-" + std::to_string(getpid()) + "-" + std::to_string(side) + ".db";
  p.pose_graph.visual = configuration("mono"); p.pose_graph.visual.tracking_enabled = true;
  p.pose_graph.visual.left.width = p.pose_graph.visual.left.height = side;
  SlamPipeline pipeline(p);
  const std::size_t bytes = std::size_t(side) * side;
  for (int i = 0; i < 70; ++i) {
    auto owner = std::make_shared<std::vector<std::uint8_t>>(bytes, 53);
    std::weak_ptr<const void> original = owner;
    cv::Mat borrowed(side, side, CV_8UC1, owner->data());
    ImageMeas image(1. + .1 * i, borrowed, owner); owner.reset();
    check(pipeline.push_image(std::move(image)), "raw image accepted into bounded owned queue");
    check(original.expired(), "owned pixel copy must release original raw message retention");
  }
  const auto count = std::min<std::size_t>(64, 32 * 1024 * 1024 / bytes);
  SlamPipelineTestAccess::checkImageQueue(pipeline, count, count * bytes);
  check(pipeline.drain().accepted == 0, "image-only stream cannot fabricate a LiDAR submap");
  pipeline.close();
}
SubmapFrame submap(std::uint64_t id, VisualFrame frame, const GaussianCloud &points, const Eigen::Isometry3d &odom) {
  auto cloud = std::make_shared<GaussianCloud>(points);
  cpu::VoxelMaps voxels;
  voxels.set_min_res(.25);
  voxels.create_voxelmaps(points.size(), [&](size_t i) { return points[i].mean; });
  LioFrame lio;
  lio.pcd = cloud;
  lio.T_odom_base = odom;
  lio.timestamp = id;
  OdomPoses poses{{double(id), odom}};
  NavigationPath navigation;
  navigation.samples.push_back({double(id), 0, 0, 0, 0, 0, 0, 1, 0});
  std::vector<VisualFrame> frames;
  frames.push_back(std::move(frame));
  return SubmapFrame(id, std::move(lio), id, id, 1, 0, 0, voxels.release_data(), std::move(poses), std::move(navigation), std::move(frames));
}
void retrievalTest(const std::string &mode, bool laser) {
  const auto config = configuration(mode);
  cv::Mat descriptors(120, 32, CV_8UC1);
  cv::RNG rng(1234);
  rng.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = Eigen::AngleAxisd(.06, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(.45, -.15, .1);
  std::vector<VisualPoint> pixels;
  GaussianCloud target, query;
  for (int i = 0; i < 120; ++i) {
    pixels.emplace_back(Eigen::Vector2f(80 + (i % 12) * 40, 65 + (i / 12) * 35), 0, 1);
    GaussianPoint g;
    g.N = 10;
    g.mean = {rng.uniform(-3.f, 3.f), rng.uniform(-3.f, 3.f), rng.uniform(-1.f, 2.f)};
    g.covariance = Eigen::Matrix3f::Identity() * .001f;
    g.regularize();
    target.push_back(g);
    g.mean = (expected.inverse() * g.mean.cast<double>()).cast<float>();
    query.push_back(g);
  }
  const std::string path = "/tmp/sapphire-visual-test-" + std::to_string(getpid()) + "-" + mode + ".db";
  {
    Memory memory(path);
    VisualSubmapIndex index(config, memory);
    for (int id = 0; id <= 4; ++id) {
      VisualFrame frame(id, mode == "stereo" ? 1 : 0);
      if (id == 0 || id == 4) frame.update_features(pixels, descriptors);
      Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
      odom.translation().x() = id * 30;
      auto map = submap(id, std::move(frame), id == 4 ? query : target, odom);
      memory.saveSubmap(map, LocalGrid{}, buildVisualScene(map));
      check(memory.loadVisualFrames(id).front().camera_id() == (mode == "stereo" ? 1u : 0u), "camera identity survives storage");
      memory.saveSubmapPose(id + 1, odom.cast<float>());
      index.addSubmap(map);
      if (id < 4)
        check(index.query(id).empty(), "adjacent/current scenes excluded before Faiss NNDR");
      else {
        const auto matches = index.query(id);
        check(matches.size() == 1 && matches[0].target_id == 0, "image retrieval survives 120 metre odometry drift");
        auto batch = prepareLoopCandidates(memory, id, odom, matches);
        check(batch.candidates.size() == 1 && batch.candidates[0].visual, "visual candidate survives spatial nonoverlap");
        if (laser) {
          auto &candidate = batch.candidates[0];
          gpu::LocalSearchTarget search{candidate.target_id, &candidate.target_voxelmaps, candidate.target_T_query_initial};
          const auto coarse = alignLoopBbs(*batch.query_cloud, {search});
          check(coarse.accepted, "image-only retrieval reaches BBS without a visual pose");
          const auto refined = refineLoopGicp(batch.query_cloud, memory.loadCloud(0), coarse.T_target_query);
          check(refined.accepted && (refined.T_target_query.matrix() - expected.matrix()).norm() < .01, "BBS reaches precise laser geometry");
        }
      }
      index.finishQuery(id);
    }
    check(!memory.loadVisualScene(0).empty(), "retrieval scene persisted");
  }
  std::remove(path.c_str());
}
void groundTest() {
  GaussianCloud floor_cloud;
  for (int y = -30; y <= 30; ++y)
    for (int x = -30; x <= 30; ++x) {
      GaussianPoint p;
      p.mean = {x * .25f + .125f, y * .25f + .125f, -1.f};
      p.N = 20;
      p.is_plane = true;
      p.covariance = Eigen::Vector3f(.005f, .005f, .0001f).asDiagonal();
      p.regularize();
      floor_cloud.push_back(p);
    }
  std::vector<mapping::GroundPatch> patches;
  mapping::collectGroundCapePatches(floor_cloud, Eigen::Isometry3d::Identity(), .05, patches);
  mapping::FlatGroundReference floor;
  check(floor.update(patches, .05, .3) && std::abs(floor.observedZ() + 1) < .01, "CAPE observes laser floor without any images");
  GaussianCloud wall;
  for (const auto &p : floor_cloud) {
    auto q = p;
    q.mean = {3, p.mean.x(), p.mean.y()};
    q.covariance = Eigen::Vector3f(.0001f, .005f, .005f).asDiagonal();
    q.regularize();
    wall.push_back(q);
  }
  patches.clear();
  mapping::collectGroundCapePatches(wall, Eigen::Isometry3d::Identity(), .05, patches);
  check(!floor.update(patches, .05, .3), "vertical wall cannot create floor evidence");
  NaviMapParameters settings;
  settings.ground_margin = .15;
  LocalGridMaker maker(settings);
  GaussianCloud cloud;
  for (float z : {0.f, .4f, 1.2f}) {
    GaussianPoint p;
    p.mean = {2, 0, z};
    cloud.push_back(p);
  }
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.translation().z() = 1;
  LocalGrid grid;
  maker.createLocalMap(cloud, pose, grid, 1.f);
  check(grid.groundCells.size() == 1 && grid.obstacleCells.size() == 1, "classification relative to observed floor");
  maker.createLocalMap(cloud, pose, grid);
  check(grid.groundCells.empty(), "unobserved floor cannot create free space");
}
void independentGeometryTest(bool appearance, int rejection = 0) {
  reject_stage = rejection; coarse_attempts = 0; refined_attempts = 0;
  PoseGraphParameters config; config.visual = configuration("mono");
  config.visual.global_xy_window = .5; config.visual.global_z_window = .5;
  NaviMapParameters nav; nav.enabled = false;
  const std::string path = "/tmp/sapphire-independent-loop-" + std::to_string(getpid()) + (appearance ? "-fused-" : "-occluded-") + std::to_string(rejection) + ".db";
  cv::RNG rng(19317); cv::Mat descriptors(120, 32, CV_8UC1); rng.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = Eigen::AngleAxisd(.06, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(rejection ? .45 : 12, -.15, .1);
  GaussianCloud target, query;
  std::vector<VisualPoint> pixels;
  for (int i = 0; i < 120; ++i) {
    GaussianPoint p; p.N = 10;
    p.mean = {rng.uniform(-3.f, 3.f), rng.uniform(-3.f, 3.f), rng.uniform(-1.f, 2.f)};
    p.covariance = Eigen::Matrix3f::Identity()*.001f; p.regularize(); target.push_back(p);
    p.mean = (expected.inverse()*p.mean.cast<double>()).cast<float>(); query.push_back(p);
    pixels.emplace_back(Eigen::Vector2f(80+(i%12)*40,65+(i/12)*35),0,1);
  }
  {
    PoseGraphBackend backend(config, nav, path, {});
    for (int id = 0; id <= 4; ++id) {
      VisualFrame image(id);
      if (id == 0 || (id == 4 && appearance)) image.update_features(pixels, descriptors.clone());
      Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
      odom.translation().x() = id*40;
      if (id == 4) odom = expected;
      backend.addFrame(submap(id, std::move(image), id == 4 ? query : target, odom));
    }
    backend.finish();
  }
  check(coarse_attempts == 1, "one target has exactly one BBS solve, including after rejection");
  check(refined_attempts == (rejection == 1 ? 0 : 1), "GICP follows only the single accepted BBS seed");
  {
    MapDatabase database(path, "resume", map_config_identity(config,nav));
    const auto links = database.loadGraphLinks();
    auto loop = std::find_if(links.begin(),links.end(),[](const auto &l){return l.type==1 && l.from_id==5 && l.to_id==1;});
    if (rejection) check(loop == links.end(), "rejected candidate is not retried using another search path");
    else {
      check(loop != links.end(), appearance ? "visual retrieval must not erase a valid spatial registration" :
            "missing visual evidence must not veto a spatial LiDAR loop");
      check((loop->transform.cast<double>().matrix()-expected.inverse().matrix()).norm()<.02,
            "geometry loop persisted in query-to-target factor direction");
    }
    database.finish();
  }
  std::remove(path.c_str());
  std::cout << "independent geometry appearance=" << appearance << " injected_rejection=" << rejection
            << " BBS=" << coarse_attempts << " GICP=" << refined_attempts << '\n';
}
void metricLoopTest(bool fused, int rejection = 0, bool metric = true, bool posed = true, bool projected = true) {
  reject_stage = rejection; coarse_attempts = refined_attempts = pnp_attempts = scored_seeds = 0;
  PoseGraphParameters config; config.visual = configuration("stereo");
  config.visual.right.intrinsics = {350, 330, 310, 230};
  const std::size_t camera_id = fused ? 1 : 0;
  const auto &K = config.visual.camera(camera_id).intrinsics;
  const RectifiedProjection projection{640,480,{K[0],K[1],K[2],K[3]}};
  NaviMapParameters nav; nav.enabled = false;
  const std::string path = "/tmp/sapphire-metric-loop-" + std::to_string(getpid()) + "-" + std::to_string(fused) +
      "-" + std::to_string(rejection) + "-" + std::to_string(metric) + "-" + std::to_string(posed) + ".db";
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity(), original = expected, target_camera = expected, query_camera = expected;
  expected.linear() = (Eigen::AngleAxisd(.14, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.06, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(.09, Eigen::Vector3d::UnitX())).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(.45, -.15, .1);
  original.linear() = Eigen::AngleAxisd(.11,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  original.translation() = Eigen::Vector3d(12,-3,1);
  target_camera.linear() = Eigen::AngleAxisd(.07,Eigen::Vector3d::UnitX()).toRotationMatrix();
  target_camera.translation() = Eigen::Vector3d(.18,-.08,.04);
  query_camera.linear() = Eigen::AngleAxisd(-.04,Eigen::Vector3d::UnitY()).toRotationMatrix();
  query_camera.translation() = Eigen::Vector3d(.3,.12,.08); // image-time motion, not static extrinsics
  auto query_odom = original * expected;
  if (!fused) query_odom.translation().x() += 100;
  cv::RNG rng(93051); cv::Mat descriptors(120,32,CV_8UC1); rng.fill(descriptors,cv::RNG::UNIFORM,0,256);
  std::vector<VisualPoint> target_pixels, query_pixels;
  for (int i=0;i<120;++i) {
    const double u=100+(i%12)*36, v=95+(i/12)*30, z=rng.uniform(4.,7.);
    // Independent synthetic visual landmarks, never projected/associated Gaussian points.
    const Eigen::Vector3d pc((u-K[2])*z/K[0],(v-K[3])*z/K[1],z);
    const Eigen::Vector3d xyz = target_camera * pc;
    VisualPoint target({float(u),float(v)},0,1);
    if (metric) {
      std::array<VisualRayObservation,3> observations;
      for(int j=0;j<3;++j) {
        Eigen::Isometry3d camera = original * target_camera;
        camera.translation() += original.linear() * Eigen::Vector3d((j-2)*.2,.02*(j-2),0);
        const Eigen::Vector3d ray = camera.inverse() * (original * xyz);
        observations[j]={-.02+.01*j,camera,ray.head<2>()/ray.z()};
      }
      auto geometry=triangulateVisualTrack(observations,K[0],K[1],{});
      check(geometry.has_value(),"visual-only synthetic triangulation supplies target depth");
      target.set_tracking(i+1,geometry);
    }
    target_pixels.push_back(target);
    const Eigen::Vector3d pq = query_camera.inverse() * expected.inverse() * xyz;
    query_pixels.emplace_back(Eigen::Vector2f(float(K[0]*pq.x()/pq.z()+K[2]),float(K[1]*pq.y()/pq.z()+K[3])),0,1);
  }
  if (rejection == 4) {
    std::mt19937 shuffle(12345);
    std::shuffle(query_pixels.begin(),query_pixels.end(),shuffle); // no PnP consensus, spatial LiDAR remains correct
  }
  GaussianCloud target_cloud, query_cloud;
  for(int i=0;i<360;++i) {
    GaussianPoint point; point.N=20;
    point.mean={rng.uniform(-3.f,3.f),rng.uniform(-3.f,3.f),rng.uniform(-1.f,2.f)};
    point.covariance=Eigen::Vector3f(.001f,.01f,.04f).asDiagonal(); point.regularize(); target_cloud.push_back(point);
    point.mean=(expected.inverse()*point.mean.cast<double>()).cast<float>();
    point.covariance=(expected.linear().transpose()*point.covariance.cast<double>()*expected.linear()).cast<float>();
    query_cloud.push_back(point);
  }
  {
    PoseGraphBackend backend(config,nav,path,{});
    for(int id=0;id<=4;++id) {
      auto odom=original; odom.translation().x()+=40*id; if(id==4) odom=query_odom;
      VisualFrame image(id,camera_id);
      if(id==0 || id==4) {
        image.update_features(id==0?target_pixels:query_pixels,descriptors.clone());
        if(projected) image.set_projection(projection);
        if(id==0 || posed) image.set_observation_pose(odom*(id==0?target_camera:query_camera));
      }
      backend.addFrame(submap(id,std::move(image),id==4?query_cloud:target_cloud,odom));
    }
    backend.finish();
  }
  const bool support = metric && posed && projected, has_seed = support && rejection != 4;
  check(pnp_attempts==(support?1:0) && scored_seeds==(has_seed?1:0),"one supported PnP and fixed-pose score per target");
  check(coarse_attempts==(has_seed?0:1),"usable PnP never invokes BBS, including after geometric rejection");
  check(refined_attempts==(rejection==3?0:1),"GICP follows only accepted independent seed geometry");
  std::cout<<"metric loop fused="<<fused<<" reject="<<rejection<<" metric="<<metric<<" posed="<<posed
           <<" PnP="<<pnp_attempts<<" BBS="<<coarse_attempts<<" score="<<scored_seeds<<" GICP="<<refined_attempts<<'\n';
  reject_stage=0;
  {
    MapDatabase database(path,"resume",map_config_identity(config,nav));
    database.validateHistoricalRecords(.1f);
    const auto links=database.loadGraphLinks();
    const auto found=std::find_if(links.begin(),links.end(),[](const auto &l){return l.type==1&&l.from_id==5&&l.to_id==1;});
    if(rejection==2 || rejection==3) check(found==links.end(),"geometric rejection leaves no factor and no alternate initializer");
    else check(found!=links.end() && (found->transform.cast<double>().matrix()-expected.inverse().matrix()).norm()<.01,
               "PnP/GICP accepted transform has correct asymmetric serialized factor direction");
  }
  {
    Memory memory(path,{},config.storage,map_config_identity(config,nav),"resume");
    memory.rebuildCommittedIndexes();
    auto changed = config.visual; changed.left.intrinsics = changed.right.intrinsics = {900,850,210,170};
    VisualSubmapIndex index(changed,memory); index.restoreHistory(5);
    auto check_seed=[&](const auto &matches) {
      check(matches.size()==1 && matches[0].target_id==0,"metric target identity survives historical/transient retrieval");
      check(bool(matches[0].T_target_query)==has_seed,"only posed metric support supplies PnP");
      if(has_seed) check((matches[0].T_target_query->matrix()-expected.matrix()).norm()<1e-4,
                        "original image-time anchor pose survives optimized map changes and reopen");
    };
    check_seed(index.query(4));
    auto fresh=query_odom; fresh.translation().y()-=75;
    VisualFrame image(5,camera_id); image.update_features(query_pixels,descriptors.clone());
    if(projected) image.set_projection(projection);
    if(posed) image.set_observation_pose(fresh*query_camera);
    auto query=submap(5,std::move(image),query_cloud,fresh);
    check_seed(index.queryTransient(query));
    memory.finish();
  }
  std::remove(path.c_str());
}
int main(int argc, char **) {
  retrievalTest("mono", argc > 1);
  retrievalTest("stereo", argc > 1);
  groundTest();
  if (argc == 1) { imageQueueOwnershipTest(320); imageQueueOwnershipTest(1024); }
  if (argc > 1) {
    independentGeometryTest(false); independentGeometryTest(true);
    independentGeometryTest(true, 1); independentGeometryTest(true, 2);
    metricLoopTest(false); metricLoopTest(true);
    metricLoopTest(true,3); metricLoopTest(true,2); metricLoopTest(true,4);
    metricLoopTest(true,0,false); metricLoopTest(true,0,true,false); metricLoopTest(true,0,true,true,false);
  }
  std::cout << "visual mapping tests passed\n";
}
