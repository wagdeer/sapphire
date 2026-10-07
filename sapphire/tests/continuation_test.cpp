#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <sys/wait.h>
#include <unistd.h>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <set>
#include <thread>
#include "backend/registration/loop_closure.hpp"
#include "backend/storage/map_database.hpp"
#include "backend/storage/memory.hpp"
#include "backend/graph/pose_graph.hpp"
#include "pipeline.hpp"
#include "backend/grid/occ_layer.hpp"
#include "backend/visual/feature/scene_features.hpp"
#include "backend/visual/feature/visual_tracker.hpp"
#include "backend/graph/loop_policy.hpp"
thread_local std::function<void()> *capture_allocation_probe = nullptr;
thread_local int allocations_until_failure = -1;
void *operator new(std::size_t bytes) {
  // One-shot, thread-owned UUID-copy probe. Clear before callback allocations.
  if (capture_allocation_probe && bytes == 37) {
    auto *probe = capture_allocation_probe; capture_allocation_probe = nullptr; (*probe)();
  }
  if (allocations_until_failure >= 0 && allocations_until_failure-- == 0) {
    allocations_until_failure = -1; throw std::bad_alloc();
  }
  if (void *p = std::malloc(bytes ? bytes : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
using namespace sapphire;
namespace fs = std::filesystem;
void check(bool ok, const std::string &message) { if (!ok) throw std::runtime_error(message); }
namespace sapphire { struct MapDatabaseTestAccess { static sqlite3 *handle(MapDatabase &db) { return db.db_; } }; }
Eigen::Isometry3d anchor(int i) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = (Eigen::AngleAxisd(.31 + .005 * i, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.17, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(.12, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  p.translation() = Eigen::Vector3d(10 + .4 * i, -2 + .1 * i, .5 + .02 * i);
  return p;
}
std::size_t grid_empty_points = 3;
LocalGrid grid(bool large = false) {
  LocalGrid g(.1f);
  g.viewPoint = {.1f, .2f, .3f};
  g.groundCells = {{0, 0}, {.1f, 0}};
  g.obstacleCells = {{.2f, 0}};
  g.emptyCells.assign(large ? 250000 : grid_empty_points, GridPoint(.3f, .1f));
  return g;
}
Eigen::Isometry3d relation() {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = (Eigen::AngleAxisd(.31, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.08, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(.12, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  p.translation() = Eigen::Vector3d(.6, -.4, .2);
  return p;
}
Eigen::Isometry3d freshAnchor() {
  auto p = anchor(0).inverse();
  p.translation() = Eigen::Vector3d(-42, 19, 3);
  return p;
}
bool metric_fixture = false;
SubmapFrame frame(int id, const Eigen::Isometry3d &a, bool target = false, int count = 120, bool mismatch = false) {
  auto cloud = std::make_shared<GaussianCloud>();
  const auto transform = target ? relation() : Eigen::Isometry3d::Identity();
  for (int i = 0; i < count; ++i) {
    const int x = i % 5, y = (i / 5) % 6, z = i / 30;
    GaussianPoint p;
    p.N = 20;
    p.voxel_key.x = i;
    p.mean = (transform * Eigen::Vector3d(.37 * x + .03 * y * y, .43 * y + .02 * x * z, .39 * z + .02 * x * y)).cast<float>();
    const Eigen::Matrix3d covariance = Eigen::Vector3d(.001, .02, .04).asDiagonal();
    p.covariance = (transform.linear() * covariance * transform.linear().transpose()).cast<float>();
    p.regularize();
    cloud->push_back(p);
  }
  cpu::VoxelMaps pyramid;
  pyramid.create_voxelmaps(cloud->size(), [&](size_t i) { return (*cloud)[i].mean; });
  LioFrame lio;
  lio.timestamp = id + 1;
  lio.T_odom_base = a;
  lio.pcd = cloud;
  NavigationPath path;
  for (int i = 0; i < 5; ++i) path.samples.push_back({double(id + 1) + .01 * i, .1f * i, 0, 0, 0, 0, 0, 1, .1f * i});
  path.spline = fit_navigation_spline(path.samples, 0, .1);

  VisualFrame image(id + 1, 0);
  cv::Mat descriptors(96, 32, CV_8UC1);
  cv::RNG random(900 + id);
  random.fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  std::vector<VisualPoint> points;
  for (int i = 0; i < 96; ++i) points.emplace_back(Eigen::Vector2f(i + 10, i + 20), i % 4, .5f);
  if (metric_fixture) {
    Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
    camera.linear() = (Eigen::AngleAxisd(.14, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-.08, Eigen::Vector3d::UnitY())).toRotationMatrix();
    camera.translation() = Eigen::Vector3d(.2, -.1, .07);
    image.set_observation_pose(a * camera);
    image.set_projection({640,480,{300,300,320,240}});
    for (int i = 0; i < 96; ++i) {
      const Eigen::Vector3d xyz((points[i].pixel().x()-320)/300.0*4, (points[i].pixel().y()-240)/300.0*4, 4);
      std::array<VisualRayObservation,3> rays;
      for (int j=0;j<3;++j) {
        Eigen::Isometry3d pose = a * camera;
        pose.translation() += a.linear() * Eigen::Vector3d((j-2)*.2, .02*(j-2), 0);
        const Eigen::Vector3d pc = pose.inverse() * (a * camera * xyz);
        rays[j] = {double(id+1)+.001*(j-2),pose,pc.head<2>()/pc.z()};
      }
      const auto geometry = triangulateVisualTrack(rays,300,300,{});
      check(geometry.has_value(), "metric persistence fixture triangulates from visual rays");
      points[i].set_tracking(i+1,geometry);
    }
  }
  image.update_features(std::move(points), descriptors);
  std::vector<VisualFrame> visuals;
  visuals.push_back(std::move(image));
  auto later = a * relation();
  OdomPoses odometry{{double(id + 1), a}, {double(id + 1) + .02, later}};
  if (mismatch) odometry.front().T_odom_base.translation().x() += 1;
  SubmapFrame f(id, std::move(lio), id, id + 1, 2, 0, 0, pyramid.release_data(), std::move(odometry), std::move(path), std::move(visuals));
  f.attach_tag(MetaTag(MetaTagType::kBinary, descriptors.row(0).clone()));
  f.attach_tag(MetaTag(MetaTagType::kFloat, cv::Mat(1, 4, CV_32FC1, cv::Scalar(.4f))));
  return f;
}
PoseGraphParameters resumeConfig() {
  PoseGraphParameters p;
  p.map_mode = "resume";
  return p;
}
void seed(const fs::path &path, bool drift = true, int nodes = 6) {
  MapDatabase db(path.string());
  std::vector<GraphLink> links;
  std::vector<std::pair<int, Eigen::Isometry3f>> poses;
  for (int i = 0; i < nodes; ++i) {
    auto f = frame(i, anchor(i), true);
    db.saveSubmap(f, grid(), buildVisualScene(f)->encode());
    Eigen::Isometry3f committed = anchor(i).cast<float>();
    if (drift && i == 3) committed.translation().x() += .004f;
    poses.emplace_back(i + 1, committed);
    if (i) links.push_back({i, i + 1, 0, (anchor(i - 1).inverse() * anchor(i)).cast<float>()});
  }
  db.saveSubmapPoses(poses, links);
  db.finish();
}

namespace {
using FaultHook = std::function<void(const char *, void *)>;
std::mutex hook_mutex;
FaultHook hook;
void setHook(FaultHook next) {
  // Build/copy next before locking; swap is noexcept. Destroy the old callback
  // after the lock guard leaves scope, including when clearing the hook.
  std::lock_guard<std::mutex> lock(hook_mutex);
  hook.swap(next);
}
FaultHook acquireHook() {
  std::lock_guard<std::mutex> lock(hook_mutex);
  return hook; // If copying throws, RAII releases the management lock.
}
} // namespace
std::mutex events_mutex;
std::condition_variable events_cv;
std::map<std::string, int> events;
std::atomic_bool ambiguous_commit{false};
thread_local bool profile_writer=false;
thread_local std::chrono::steady_clock::time_point writer_start, begin_start;
thread_local double prebegin_ms=0;
namespace sapphire::database_detail {
void writableStorageTestPoint(const char *point, void *context) {
  if (std::string(point)=="b2-before-w") { profile_writer=true; writer_start=std::chrono::steady_clock::now(); }
  const auto callback = acquireHook();
  // A snapshot owns its callable through invocation. Never hold hook_mutex
  // across a blocking Barrier or an intentionally thrown fault.
  if (callback) callback(point, context);
  { std::lock_guard<std::mutex> lock(events_mutex); ++events[point]; }
  events_cv.notify_all();
}
}
extern "C" int __real_sqlite3_exec(sqlite3 *, const char *, int (*)(void *, int, char **, char **), void *, char **);
extern "C" int __wrap_sqlite3_exec(sqlite3 *db, const char *q, int (*cb)(void *, int, char **, char **), void *arg, char **msg) {
  if(profile_writer && !std::strcmp(q,"BEGIN IMMEDIATE;")) {
    begin_start=std::chrono::steady_clock::now();prebegin_ms=std::chrono::duration<double,std::milli>(begin_start-writer_start).count();
  }
  const int rc = __real_sqlite3_exec(db, q, cb, arg, msg);
  if(profile_writer && !std::strcmp(q,"COMMIT;")) {
    const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin_start).count();
    std::cout<<"B2_W pre_BEGIN_ms="<<prebegin_ms<<" BEGIN_COMMIT_ms="<<ms<<'\n';profile_writer=false;
  }
  if (!std::strcmp(q, "COMMIT;") && ambiguous_commit.exchange(false)) return SQLITE_IOERR;
  return rc;
}
void awaitEvent(const std::string &event, int count = 1) {
  std::unique_lock<std::mutex> lock(events_mutex);
  check(events_cv.wait_for(lock, std::chrono::seconds(30), [&] { return events[event] >= count; }), "timed out: " + event);
}
void clearEvents() { std::lock_guard<std::mutex> lock(events_mutex); events.clear(); }
struct Barrier {
  std::mutex m; std::condition_variable cv; bool entered = false, released = false;
  void arrive() { std::unique_lock<std::mutex> l(m); entered = true; cv.notify_all(); cv.wait(l, [&] { return released; }); }
  void wait() { std::unique_lock<std::mutex> l(m); check(cv.wait_for(l, std::chrono::seconds(30), [&] { return entered; }), "barrier timed out"); }
  void release() { std::lock_guard<std::mutex> l(m); released = true; cv.notify_all(); }
};
Eigen::Isometry3d continuedAnchor(int i) {
  auto a = freshAnchor(); a.translation() += Eigen::Vector3d(20 * i, -7 * i, 0); return a;
}
void attach(PoseGraphBackend &b, int target) {
  auto result = b.attachFreshSession(frame(40, freshAnchor()), grid(), target, relation(), 77);
  check(result.status == AttachmentStatus::Attached, "real B1 attachment: " + result.message);
  b.beginContinuation(77);
}
void submit(PoseGraphBackend &b, int index) {
  auto q = frame(40 + index, continuedAnchor(index)); auto g = grid();
  check(b.submitContinuation(q, g, 77) == ContinuationAdmission::Accepted, "B2 accepted");
}
void policy() {
  static_assert(loop_policy::temporal_exclusion == 3);
  for (int mask = 0; mask < 512; ++mask) {
    std::vector<std::uint64_t> roots(11);
    for (int i = 1; i < 11; ++i) roots[i] = (mask & (1 << (i - 1))) ? i : roots[i - 1];
    for (int q = 1; q < 11; ++q) for (int t = 0; t < 11; ++t) {
      bool expected = t < q && (roots[q] != roots[t] || q - t > 3);
      check(loop_policy::eligible(q,t,roots[q],roots[t],q) == expected, "Policy C partition oracle");
      check((t < loop_policy::prefix(q, roots[q])) == expected, "prefix equivalence");
    }
  }
  check(loop_policy::prefix(100,99) == 99 && loop_policy::prefix(101,99) == 99 &&
        loop_policy::prefix(102,99) == 99 && loop_policy::prefix(103,99) == 100, "SQL root100/query101 conversions");
}
std::string semantic(PoseGraphBackend &b, int count) {
  std::ostringstream out;
  out << b.graphRevision() << ':';
  for (int i=0;i<count;++i) {
    const auto p = *b.committedPose(i);
    out.write(reinterpret_cast<const char *>(p.data()), 64);
    auto candidates=b.historicalSpatialCandidates(i);
    std::vector<std::uint64_t> ids; for (auto &m:candidates) ids.push_back(m.submap_id);
    std::sort(ids.begin(),ids.end()); for(auto id:ids) out << id << ','; out << ';';
  }
  auto transient=frame(count, continuedAnchor(count));
  const auto matches=b.transientVisualCandidates(transient);
  for(const auto &m:matches) out<<m.target_id<<':'<<m.matches<<':'<<m.retrieval_score<<';';
  const auto metadata=b.historicalVisualMetadata();
  for(const auto &[id,tag]:metadata) out<<id<<':'<<tag.first<<':'<<tag.second<<';';
  const auto occupancy=b.historicalOccupancy();
  std::map<std::pair<int,int>,std::tuple<bool,bool,int>> cells;
  occupancy.visitEvidence([&](int x,int y,bool observed,bool occupied,int owner){cells[{x,y}]={observed,occupied,owner};});
  for(const auto &[xy,c]:cells) out<<xy.first<<','<<xy.second<<':'<<std::get<0>(c)<<std::get<1>(c)<<','<<std::get<2>(c)<<';';
  const auto map=occupancy.getMap(); out<<map.width<<','<<map.height<<','<<map.originX<<','<<map.originY<<';';
  out.write(reinterpret_cast<const char *>(map.cells.data()), map.cells.size());
  return out.str();
}
void oracle(const fs::path &path, const fs::path &expected, int count) {
  const auto child=fork(); check(child>=0,"fork oracle");
  if(!child) {
    setenv("CUDA_VISIBLE_DEVICES","",1);
    execl("/proc/self/exe","sapphire_continuation_test","--read",path.c_str(),expected.c_str(),std::to_string(count).c_str(),nullptr);
    _exit(120);
  }
  int status=0; check(waitpid(child,&status,0)==child && status==0,"independent CUDA-hidden A2 oracle");
}
void success(const fs::path &path, int n) {
  seed(path,true,n);
  std::string expected; std::uint64_t revision;
  {
    PoseGraphBackend b(resumeConfig(),{},path.string(),[](auto){throw std::runtime_error("B3 callback");});
    attach(b,n); revision=b.graphRevision(); clearEvents();
    auto wrong=frame(41,continuedAnchor(1));auto invalid=grid();
    check(b.submitContinuation(wrong,invalid,78,false)==ContinuationAdmission::WrongProducer,"generation refusal retains producer work");
    auto outOfOrder=frame(42,continuedAnchor(2));
    check(b.submitContinuation(outOfOrder,invalid,77,false)==ContinuationAdmission::WrongSequence,"sequence refusal");
    invalid.emptyCells.resize(64*1024*1024/sizeof(GridPoint));
    check(b.submitContinuation(wrong,invalid,77,false)==ContinuationAdmission::Oversize&&wrong.lio().pcd,"oversize frozen refusal before transfer");
    invalid=grid();
    auto reserved=frame(41,continuedAnchor(1));reserved.visual_frames().clear();
    reserved.visual_frames().reserve(64*1024*1024/sizeof(VisualFrame)+1);
    check(b.submitContinuation(reserved,invalid,77,false)==ContinuationAdmission::Oversize&&reserved.lio().pcd,
          "empty visual container reserved capacity is bounded before transfer");
    reserved.visual_frames().shrink_to_fit();
    auto roi=frame(41,continuedAnchor(1));roi.visual_frames().clear();
    cv::Mat backing(1,64*1024*1024+32,CV_8UC1);
    VisualFrame roi_frame(42,0);
    roi_frame.update_features({VisualPoint(Eigen::Vector2f(1,1),0,.5f)},backing.colRange(0,32));
    roi.visual_frames().push_back(std::move(roi_frame));
    check(b.submitContinuation(roi,invalid,77,false)==ContinuationAdmission::Oversize&&roi.lio().pcd,
          "tiny descriptor ROI retaining a large parent is refused before transfer");
    roi.visual_frames().clear();backing.release();
    const auto workload_start=std::chrono::steady_clock::now();
    for(int i=1;i<=5;++i) submit(b,i);
    const auto admitted=std::chrono::steady_clock::now();
    awaitEvent("b2-completed",5);
    const double admitted_seconds=std::chrono::duration<double>(admitted-workload_start).count();
    const double completed_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-workload_start).count();
    std::cout<<"B2_WORKLOAD historical_nodes="<<n<<" grid_empty_points="<<grid_empty_points<<" offered=5 admitted=5 completed=5 producer_offer_and_admit_rate="<<5/admitted_seconds
             <<" completed_rate="<<5/completed_seconds<<" high_water="<<b.continuationProgress().high_water<<'\n';
    {std::lock_guard<std::mutex> lock(events_mutex);check(events["runtime-full-rebuild"]==0,"zero ordinary full runtime rebuilds");}
    auto p=b.continuationProgress();
    check(p.completed==5 && p.ready_revision==revision+5 && p.high_water<=1,"bounded completion/revision");
    const auto correction=b.committedPose(n+5)->cast<double>()*continuedAnchor(5).inverse();
    check(b.T_map_odom().matrix().isApprox(correction.matrix(),1e-12),"latest-active float correction");
    expected=semantic(b,n+6);
    std::ofstream out(path.string()+".oracle",std::ios::binary); out.write(expected.data(),expected.size()); out.close();
    b.finish();
  }
  oracle(path,path.string()+".oracle",n+6);
  MapDatabase db(path.string(),"resume"); db.validateHistoricalRecords(.1f);
  auto links=db.loadGraphLinks(); int rootOdom=0, rootLoop=0;
  for(const auto &l:links) { if(l.type==0 && l.to_id==n+1) ++rootOdom; if(l.type==1&&l.from_id==n+1) ++rootLoop; }
  check(!rootOdom && rootLoop==1,"Q0 unique B1 root");
  check(db.committedRevision()==revision+5,"five finalized W commits");
  db.finish();
}
void failureCase(const fs::path &path, const std::string &fault, bool nonstandard=false) {
  seed(path,false,6);
  Barrier barrier; // Outlives the backend worker and any callback snapshot.
  PoseGraphBackend b(resumeConfig(),{},path.string(),{});
  attach(b,6); const auto old=b.graphRevision(); clearEvents();
  setHook([&barrier, fault, nonstandard](const char *point,void *) {
    if(std::string(point)=="b2-before-preparation") barrier.arrive();
    if(std::string(point)==fault) { if(nonstandard) throw 73; throw std::runtime_error("retained B2 test cause"); }
  });
  submit(b,1); barrier.wait(); submit(b,2);
  auto blocked=std::async(std::launch::async,[&]{auto q=frame(43,continuedAnchor(3));auto g=grid();return b.submitContinuation(q,g,77);});
  awaitEvent("b2-capacity-wait",3);
  if(fault=="unknown") ambiguous_commit.store(true);
  barrier.release();
  check(blocked.get()==ContinuationAdmission::Failed,"blocked producer wakes on sticky failure");
  bool caught=false; try{b.finish();}catch(...){caught=true;}
  check(caught&&b.failed(),"checked finish retains failure"); setHook({});
  auto p=b.continuationProgress(); check(!p.completed&&p.ready_revision==old&&p.canceled_first==41&&p.canceled_last==42,"no Qn+1 and canceled tail accounting");
  check(!b.hasActiveCorrection(),"failed correction unavailable");
  const bool committed=fault=="b2-postcommit-materialization"||fault=="b2-before-visual"||fault=="b2-before-occupancy"||fault=="b2-runtime-ready"||fault=="after-commit";
  if(committed) {
    check(p.last_commit_outcome==CommitOutcome::Committed,"known committed outcome");
    check(p.committed_revision==old+1&&p.committed_outcome_known,"B3 committed mirror leads retained Y after materialization failure");
  }
  else if(fault=="unknown") check(p.last_commit_outcome==CommitOutcome::Unknown&&!p.committed_outcome_known,"unknown outcome preserved; last known C is unresolved");
  else check(p.last_commit_outcome==CommitOutcome::NotCommitted,"not committed outcome");
  MapDatabase recovery(path.string(),"recover"); recovery.validateHistoricalRecords(.1f);
  check(recovery.committedRevision()==old+((fault=="unknown"||committed)?1:0),"DB authoritative after failure"); recovery.finish();
  PoseGraphBackend reconstructed(resumeConfig(),{},path.string(),{}); check(!reconstructed.hasActiveCorrection(),"recovery has no live domain"); reconstructed.finish();
}
void setDescriptors(SubmapFrame &q, const cv::Mat &bytes) {
  VisualFrame image(q.lio().timestamp,0);
  std::vector<VisualPoint> points;
  for(int i=0;i<bytes.rows;++i) points.emplace_back(Eigen::Vector2f(i+1,i+2),0,.5f);
  image.update_features(std::move(points),bytes.clone());
  q.visual_frames().clear(); q.visual_frames().push_back(std::move(image));
}
void topology(const fs::path &root) {
  for (int distance=1;distance<=4;++distance) {
    const auto path=root/("same-chain-"+std::to_string(distance)+".db"); seed(path,false,1);
    MapDatabase db(path.string(),"resume"); db.validateHistoricalRecords(.1f); db.promoteToWritable(.1f);
    auto previous=freshAnchor(); auto pose=anchor(0)*relation();
    auto q0=frame(1,previous);
    auto revision=db.commitFinalizedSubmap(q0,grid(),buildVisualScene(q0)->encode(),{{2,pose.cast<float>()}},
       {2,1,1,relation().inverse().cast<float>()},{},db.mapUuid(),db.committedRevision(),2,map_config_identity({},{}),2);
    bool rejected=false;
    for(int d=1;d<=distance;++d) {
      auto current=previous; current.translation().x()+=.1;
      const Eigen::Isometry3d next=pose*previous.inverse()*current;
      auto q=frame(1+d,current);
      std::optional<GraphLink> loop;
      if(d==distance) loop=GraphLink{2+d,2,1,(next.inverse()*(anchor(0)*relation())).cast<float>()};
      try {revision=db.commitFinalizedSubmap(q,grid(),buildVisualScene(q)->encode(),{{2+d,next.cast<float>()}},
            {1+d,2+d,0,(previous.inverse()*current).cast<float>()},loop,db.mapUuid(),revision,2+d,map_config_identity({},{}),2);}
      catch(const MapError&){rejected=true;}
      previous=current; pose=next;
    }
    check(rejected==(distance<=3),"W same-chain distance 1/2/3 rejected, 4 accepted");
    try{db.finish();}catch(...){}
    MapDatabase reopened(path.string(),"recover");reopened.validateHistoricalRecords(.1f);reopened.finish();
  }
  const auto path=root/"cross-chain.db";seed(path,false,99);
  {
    MapDatabase db(path.string(),"resume");db.validateHistoricalRecords(.1f);db.promoteToWritable(.1f);
    auto rootPose=anchor(98)*relation();auto q0=frame(99,freshAnchor());
    auto revision=db.commitFinalizedSubmap(q0,grid(),buildVisualScene(q0)->encode(),{{100,rootPose.cast<float>()}},
      {100,99,1,relation().inverse().cast<float>()},{},db.mapUuid(),db.committedRevision(),100,map_config_identity({},{}),100);
    auto q=frame(100,continuedAnchor(1)); const auto delta=freshAnchor().inverse()*continuedAnchor(1);
    const Eigen::Isometry3d next=rootPose*delta;
    db.commitFinalizedSubmap(q,grid(),buildVisualScene(q)->encode(),{{101,next.cast<float>()}},
      {100,101,0,delta.cast<float>()},GraphLink{101,99,1,(next.inverse()*anchor(98)).cast<float>()},
      db.mapUuid(),revision,101,map_config_identity({},{}),100);db.finish();
  }
  const auto child=fork();check(child>=0,"topology child");
  if(!child){setenv("CUDA_VISIBLE_DEVICES","",1);execl("/proc/self/exe","sapphire_continuation_test","--topology-read",path.c_str(),nullptr);_exit(120);}
  int status;check(waitpid(child,&status,0)==child&&status==0,"writer close -> independent A2 nearby-ID cross-chain reopen");
}
void visualPopulation(const fs::path &root) {
  // 84*96 is Flat, 86*96 crosses the unchanged 8192-descriptor IVF boundary.
  const auto path=root/"visual-prefix.db";
  cv::Mat queryBytes(96,32,CV_8UC1); cv::RNG rng(987654);rng.fill(queryBytes,cv::RNG::UNIFORM,0,256);
  cv::Mat older=queryBytes.clone();for(int i=0;i<older.rows;++i)older.at<unsigned char>(i,0)^=1;
  {
    MapDatabase db(path.string());std::vector<GraphLink> links;
    for(int i=0;i<90;++i){
      auto f=frame(i,anchor(i));
      if(i==83)setDescriptors(f,older);
      if(i>=84)setDescriptors(f,queryBytes);
      db.saveSubmap(f,grid(),buildVisualScene(f)->encode());
      if(i==84)links.push_back({85,84,1,(anchor(84).inverse()*anchor(83)).cast<float>()});
      else if(i)links.push_back({i,i+1,0,(anchor(i-1).inverse()*anchor(i)).cast<float>()});
    }
    db.saveSubmapPoses({},links);db.finish();
  }
  Memory memory(path.string(),{}, {},map_config_identity({},{}),"resume");memory.rebuildCommittedIndexes();
  VisualLoopParameters config;config.enabled=true;
  VisualSubmapIndex index(config,memory);
  index.materializeCommittedRoot(85);index.beginContinuation(84,85);
  check(index.eligiblePrefix()==84&&index.historyMetadata().size()==84,"B1->B2 canonical prefix");
  check(!index.historyStats().ivf,"canonical prefix starts Flat");
  auto q=frame(85,anchor(85));setDescriptors(q,queryBytes);
  auto matches=index.queryTransient(q);
  check(!matches.empty()&&matches.front().target_id==83,"excluded exact nearest neighbor cannot steal pre-NNDR match");
  for(int count=86;count<=90;++count){
    const auto before=index.eligiblePrefix();index.advanceContinuation(count);
    check(index.eligiblePrefix()-before<=1,"zero/one prefix advance");
    auto query=frame(count,anchor(count));setDescriptors(query,queryBytes);
    auto active=index.queryTransient(query);
    VisualSubmapIndex independent(config,memory);independent.restoreHistory(count);
    auto reconstructed=independent.queryTransient(query);
    check(index.historyStats().ivf == independent.historyStats().ivf, "aligned Flat/IVF mode");
    check(index.historyMetadata()==independent.historyMetadata(),"same Flat/IVF population and fingerprints");
    check(active.size()==reconstructed.size(),"same settled Flat/IVF query result count");
    for(std::size_t i=0;i<active.size();++i)check(active[i].target_id==reconstructed[i].target_id&&active[i].matches==reconstructed[i].matches&&
                                                active[i].retrieval_score==reconstructed[i].retrieval_score,"same settled training and deterministic ties");
  }
  check(index.historyStats().ivf,"incremental prefix crosses to IVF");
  bool refused=false;try{index.query(85);}catch(const std::logic_error&){refused=true;}
  check(refused,"active backward query explicitly refused");memory.finish();
}
void retryAndFinish(const fs::path &root) {
  for(bool retry:{false,true}) {
    const auto path=root/(retry?"retry.db":"finish-retry.db");seed(path,false,6);
    std::atomic_bool first{true}; // Destroyed after the backend has joined.
    PoseGraphBackend b(resumeConfig(),{},path.string(),{});attach(b,6);clearEvents();
    setHook([&first](const char *stage,void*) {if(std::string(stage)=="b2-before-preparation"&&first.exchange(false))throw 73;});
    submit(b,1);awaitEvent("b2-retryable");
    check(!b.failed()&&b.continuationProgress().head==ContinuationHead::Retryable,"pre-mutation nonstandard remains retryable");
    if(retry){b.retryContinuation(41,77);awaitEvent("b2-completed");b.finish();check(b.continuationProgress().completed==1,"explicit same-head retry succeeds");}
    else {bool caught=false;try{b.finish();}catch(int e){caught=e==73;}check(caught&&b.continuationProgress().canceled_first==41,"finish reports original retry cause without deadlock");}
    setHook({});
  }
}
void finishAndReset(const fs::path &root) {
  for(bool abort:{false,true}) {
    auto path=root/(abort?"reset.db":"drain.db");seed(path,false,6);
    Barrier barrier;
    PoseGraphBackend b(resumeConfig(),{},path.string(),{});attach(b,6);clearEvents();
    setHook([&barrier](const char *stage,void*){if(std::string(stage)=="b2-before-preparation")barrier.arrive();});
    submit(b,1);barrier.wait();submit(b,2);
    auto q=frame(43,continuedAnchor(3));auto g=grid();
    check(b.submitContinuation(q,g,77,false)==ContinuationAdmission::Full&&q.lio().pcd,"try-full retains input");
    auto blocked=std::async(std::launch::async,[&]{return b.submitContinuation(q,g,77);});
    awaitEvent("b2-capacity-wait",3);
    b.stopAdmission(abort);check(blocked.get()==ContinuationAdmission::Stopped,"stop wakes blocked producer");
    auto finisher=std::async(std::launch::async,[&]{if(abort)b.resetFreshSession();else b.finish();});
    barrier.release();finisher.get();setHook({});auto p=b.continuationProgress();
    check(p.completed==(abort?1:2),"healthy finish drains; reset does not run later head");
    if(abort)check(p.canceled_first==42&&p.canceled_last==42,"reset accounts accepted tail");
  }
}

namespace sapphire {
struct SlamPipelineTestAccess {
  static void replace(SlamPipeline &p) {p.create_pose_graph();}
  static void resetDomainStores(SlamPipeline &p) {
    p.output_domain_current_.store(false);p.session_id_.fetch_add(1);
  }
  static void uniqueReplacementPath(SlamPipeline &p) {
    p.filename_ += "-replacement";fs::create_directories(fs::path(p.save_path_)/p.filename_);
  }

  static void marginal(SlamPipeline &p, MargiFrame frame) {
    std::unique_lock<std::mutex> lock(p.marginal_mutex_);
    p.marginal_cv_.wait(lock,[&]{return p.marginal_frames_.size()<2||p.failed()||p.stopping_.load();});
    check(!p.failed()&&!p.stopping_.load(),"healthy marginal producer");
    p.marginal_frames_.push_back(std::move(frame));p.marginal_cv_.notify_all();
  }
  static ContinuationProgress progress(SlamPipeline &p) {
    std::lock_guard<std::mutex> lock(p.pose_graph_mutex_);return p.pose_graph_->continuationProgress();
  }
  static void finish(SlamPipeline &p) {p.pose_graph_->finish();}
  static PoseGraphBackend &backend(SlamPipeline &p) {return *p.pose_graph_;}
  static OutputSink output(SlamPipeline &p) {return p.output_;}
  static void optionalCopies(SlamPipeline &p, std::size_t count) {
    p.trajectory_.resize(count); p.emit_local_trajectory({}, 0);
  }
  static void optionalMap(SlamPipeline &p) { p.window_count_=0; p.emit_local_map(0); }
};
}
void pipelineContinuation(const fs::path &path) {
  seed(path,false,6);clearEvents();
  SapphireParameters params;params.pose_graph=resumeConfig();params.pose_graph.database_path=path.string();
  params.pose_graph.submap_travel_distance=.001;
  SlamPipeline pipeline(params,{},std::make_pair(6,relation()));
  for(int i=0;i<4;++i){
    const auto a=i<2?freshAnchor():continuedAnchor(1);
    auto q=frame(i,a);GaussianCloud points=*q.lio().pcd;
    for(auto &point:points)point=point.transformed(a.linear(),a.translation());
    StateGroup state;state.R=a.linear();state.p=a.translation();
    SlamPipelineTestAccess::marginal(pipeline,MargiFrame(state,std::move(points),i+1,Eigen::Matrix<double,6,1>::Ones(),i*.002));
  }
  awaitEvent("b2-completed");
  auto p=SlamPipelineTestAccess::progress(pipeline);
  check(p.last_completed==1&&p.completed==1&&!pipeline.failed(),"same producer B1->B2 pipeline handoff");
  pipeline.shutdown();SlamPipelineTestAccess::finish(pipeline);
}
void newMapFinish(const fs::path &path) {
  {
    SapphireParameters params;params.pose_graph.database_path=path.string();params.pose_graph.submap_travel_distance=1000;
    SlamPipeline pipeline(params);
    const auto a=anchor(0);auto q=frame(0,a);GaussianCloud points=*q.lio().pcd;
    for(auto &point:points)point=point.transformed(a.linear(),a.translation());
    StateGroup state;state.R=a.linear();state.p=a.translation();
    SlamPipelineTestAccess::marginal(pipeline,MargiFrame(state,std::move(points),1,Eigen::Matrix<double,6,1>::Ones(),0));
    pipeline.shutdown();check(!pipeline.failed(),"new-map shutdown preserves final short builder flush");
    SlamPipelineTestAccess::finish(pipeline);
  }
  MapDatabase db(path.string(),"resume");db.validateHistoricalRecords(.1f);
  int nodes=0;db.visitHistoricalNodes([&](int,const auto &,const auto &,const auto &){++nodes;});
  check(nodes==1,"new-map accepted marginal tail persisted on healthy finish");db.finish();
}
void upstreamBounds() {
  LruCache<int> cache(2*sizeof(int));auto value=std::make_shared<const int>(17);cache.put(1,value,sizeof(int));
  allocations_until_failure=1;bool allocation_failed=false;
  try{cache.put(2,value,sizeof(int));}catch(const std::bad_alloc&){allocation_failed=true;}
  allocations_until_failure=-1;
  check(allocation_failed&&cache.stats().entries==1,"LRU map allocation failure rolls back its local list insertion");
  for(int i=3;i<=6;++i)cache.put(i,value,sizeof(int));
  check(cache.stats().entries==2&&cache.get(6)&&cache.get(5),"cache remains usable after clean preparation failure");
  Synchronizer sync(.11,8);
  for(int i=0;i<8;++i){ImuMeas m;m.timestamp=i*.02;m.gyro.setZero();m.accel.setZero();check(sync.push_imu(m),"bounded IMU accepts capacity");}
  ImuMeas next;next.timestamp=.16;next.gyro.setZero();next.accel.setZero();check(!sync.push_imu(next),"IMU refuses overflow");
  LidarPoint point;point.x=point.y=point.z=1;point.intensity=1;point.time_offset=.1;
  check(sync.push_lidar(0,{point}),"first scan");check(sync.push_lidar(.2,{point}),"second scan");
  check(!sync.push_lidar(.4,{point}),"LiDAR full refusal");
  MeasGroup group;check(sync.sync_packages(group),"drain synchronized scan");
  check(sync.push_imu(next),"capacity refusal did not advance IMU timestamp watermark");
  check(sync.push_lidar(.4,{point}),"capacity refusal did not advance LiDAR watermark");
  std::vector<LidarPoint> oversized(16*1024*1024/sizeof(LidarPoint)+1,point);
  check(!sync.push_lidar(.6,std::move(oversized)),"oversize raw packet refusal");
  oversized.resize(1); Synchronizer capacitySync(.11);
  check(!capacitySync.push_lidar(.6,std::move(oversized))&&oversized.size()==1,"retained raw capacity is bounded before ownership transfer");
}

void allKeyDelta(const fs::path &path) {
  seed(path,false,6);std::string expected;
  {
    Eigen::Matrix4f intended;
    PoseGraphBackend b(resumeConfig(),{},path.string(),{});attach(b,6);clearEvents();
    setHook([&intended](const char *stage,void *context){if(std::string(stage)=="b2-after-solver"){
      auto &values=*static_cast<gtsam::Values *>(context);auto p=values.at<gtsam::Pose3>(2).matrix();p(0,3)+=.001;
      values.update(2,gtsam::Pose3(p));intended=p.cast<float>();
    }});
    submit(b,1);awaitEvent("b2-completed");setHook({});
    check((b.committedPose(2)->matrix().array()==intended.array()).all(),"all-key reconciliation includes injected historical change outside append update");
    expected=semantic(b,8);std::ofstream out(path.string()+".oracle",std::ios::binary);out.write(expected.data(),expected.size());out.close();b.finish();
  }
  oracle(path,path.string()+".oracle",8);
}
void ordinaryLoop(const fs::path &path) {
  seed(path,false,1);Eigen::Isometry3d measured;std::string expected;
  {
    PoseGraphBackend b(resumeConfig(),{},path.string(),{});attach(b,1);clearEvents();
    setHook([&measured](const char *stage,void *context){if(std::string(stage)=="b2-after-gicp"){
      auto &r=*static_cast<GicpResult *>(context);check(r.accepted,"real ordinary GICP accepts");measured=r.T_target_query;
    }});
    auto q=frame(41,freshAnchor());auto g=grid();check(b.submitContinuation(q,g,77)==ContinuationAdmission::Accepted,"loop input accepted");
    awaitEvent("b2-completed");setHook({});
    expected=semantic(b,3);std::ofstream out(path.string()+".oracle",std::ios::binary);out.write(expected.data(),expected.size());out.close();b.finish();
  }
  oracle(path,path.string()+".oracle",3);
  MapDatabase db(path.string(),"resume");db.validateHistoricalRecords(.1f);auto links=db.loadGraphLinks();
  std::optional<GraphLink> loop;for(auto &l:links)if(l.from_id==3&&l.type==1)loop=l;
  check(loop&&loop->to_id==1,"one ordinary cross-chain near-ID loop");
  check((loop->transform.matrix().array()==measured.inverse().cast<float>().matrix().array()).all(),"exact serialized inverse noncommuting SE3");
  std::vector<Eigen::Isometry3d> poses;db.visitHistoricalNodes([&](int,const auto &,const auto &p,const auto &){poses.push_back(p.template cast<double>());});
  gtsam::BetweenFactor<gtsam::Pose3> factor(2,0,gtsam::Pose3(loop->transform.cast<double>().matrix()),gtsam::noiseModel::Unit::Create(6));
  check(factor.evaluateError(gtsam::Pose3(poses[2].matrix()),gtsam::Pose3(poses[0].matrix())).norm()<1e-5,"actual GTSAM residual for persisted inverse");
  db.finish();
}
void incrementalOwners(const fs::path &path) {
  Memory memory(path.string());std::vector<Eigen::Isometry3f> poses;std::vector<LocalGrid> grids;
  OccupancyGrid actual(NaviMapParameters{});
  for(int i=0;i<3;++i){auto a=Eigen::Isometry3d::Identity();a.translation().x()=10*i;
    auto f=frame(i,a);LocalGrid g(.1f);g.groundCells={{-.1f,-.1f}};g.emptyCells.assign(30,GridPoint(-.1f,-.1f));
    g.obstacleCells.assign(30,GridPoint(-.1f,-.1f));if(i==2)g=LocalGrid(.1f);
    memory.saveSubmap(f,g,buildVisualScene(f));if(i)memory.saveSubmapPoses({},{{i,i+1,0,Eigen::Translation3f(10,0,0)*Eigen::Isometry3f::Identity()}});
    poses.push_back(a.cast<float>());grids.push_back(g);check(actual.append(GridFrame(i+1,poses.back(),g)),"owner append");
  }
  const auto local=frame(0,Eigen::Isometry3d::Identity()).bounds();
  auto old=memory.recallSpatial(local,poses[1]);check(old.size()==1&&old[0].submap_id==1,"old region membership");
  auto qAnchor=Eigen::Isometry3d::Identity();qAnchor.translation().x()=30;auto q=frame(3,qAnchor);auto g=grid();
  poses[1].translation().x()=-10;poses.push_back(qAnchor.cast<float>());grids.push_back(g);
  std::vector<std::pair<int,Eigen::Isometry3f>> delta{{2,poses[1]},{4,poses[3]}};
  memory.commitFinalizedSubmap(q,g,buildVisualScene(q)->encode(),delta,{3,4,0,Eigen::Translation3f(10,0,0)*Eigen::Isometry3f::Identity()},
                              {},memory.mapUuid(),memory.committedRevision(),4,map_config_identity({},{}),1);
  memory.materializeCommittedDelta(q,delta);
  auto oldPose=Eigen::Isometry3f::Identity();oldPose.translation().x()=10;
  check(memory.recallSpatial(local,oldPose).empty(),"exact old AABB removed");
  auto now=memory.recallSpatial(local,poses[1]);check(now.size()==1&&now[0].submap_id==1,"new AABB inserted");
  auto update=actual.update([&](int sql,LocalGrid &grid){return memory.loadLocalGrid(sql-1,grid);});
  update-=2;update+=GridFrame(2,poses[1],grids[1]);check(update.commit(),"batch moved-owner refusion");
  check(actual.append(GridFrame(4,poses[3],g)),"append Q after replacement");
  OccupancyGrid rebuilt(NaviMapParameters{});for(int i=0;i<4;++i)check(rebuilt.append(GridFrame(i+1,poses[i],grids[i])),"canonical replay");
  check(actual.getMap().cells==rebuilt.getMap().cells&&actual.activeNodeIds()==rebuilt.activeNodeIds(),"negative/clamped/empty moved-owner replay");
  memory.finish();
}

void offeredWorkload(const fs::path &path, int n) {
  seed(path,false,n);PoseGraphBackend b(resumeConfig(),{},path.string(),{});attach(b,n);clearEvents();
  // A finite source schedule independent of backend service. Pending source identities
  // are scalar counts; the producer constructs/holds at most one frozen payload.
  constexpr int count=12;
  std::mutex mutex;std::condition_variable cv;int offered=0,admitted=0,backlog=0;
  const auto start=std::chrono::steady_clock::now();auto last_offer=start;
  std::thread source([&]{
    std::unique_lock<std::mutex> lock(mutex);
    for(int i=1;i<=count;++i){
      cv.wait_until(lock,start+std::chrono::milliseconds(i),[]{return false;});
      ++offered;last_offer=std::chrono::steady_clock::now();backlog=std::max(backlog,offered-admitted);cv.notify_all();
    }
  });
  for(int i=1;i<=count;++i){
    {std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return offered>=i;});}
    submit(b,i);
    {std::lock_guard<std::mutex> lock(mutex);++admitted;}
  }
  source.join();awaitEvent("b2-completed",count);
  const auto done=std::chrono::steady_clock::now();auto p=b.continuationProgress();
  std::cout<<"B2_OFFERED historical_nodes="<<n<<" offered="<<offered<<" admitted="<<admitted<<" completed="<<p.completed
    <<" offered_rate="<<count/std::chrono::duration<double>(last_offer-start).count()
    <<" admitted_completed_rate="<<count/std::chrono::duration<double>(done-start).count()
    <<" external_source_backlog_high_water="<<backlog<<" backend_waiting_high_water="<<p.high_water<<" final_backlog="<<offered-p.completed<<'\n';
  check(p.completed==count&&p.high_water==1,"bounded backlog with independent finite offered schedule");b.finish();
}

void workerGuard(const fs::path &path) {
  seed(path,false,6);Barrier barrier;PoseGraphBackend b(resumeConfig(),{},path.string(),{});clearEvents();
  setHook([&barrier](const char *stage,void*){if(std::string(stage)=="b2-worker-entry"){barrier.arrive();throw 91;}});
  attach(b,6);barrier.wait();submit(b,1);
  auto blocked=std::async(std::launch::async,[&]{auto q=frame(42,continuedAnchor(2));auto g=grid();return b.submitContinuation(q,g,77);});
  awaitEvent("b2-capacity-wait",2);barrier.release();check(blocked.get()==ContinuationAdmission::Failed,"final guard wakes submitter");
  bool caught=false;try{b.finish();}catch(int n){caught=n==91;}setHook({});
  const auto p=b.continuationProgress();check(caught&&p.canceled_first==41&&p.canceled_last==41,"final guard retains cause and accounts accepted tail");
}

void restartContinuation(const fs::path &path) {
  seed(path,false,6);
  { PoseGraphBackend b(resumeConfig(),{},path.string(),{});attach(b,6);clearEvents();submit(b,1);awaitEvent("b2-completed");b.finish(); }
  const auto child=fork();check(child>=0,"fork restart continuation");
  if(!child){execl("/proc/self/exe","sapphire_continuation_test","--restart-continue",path.c_str(),nullptr);_exit(120);}
  int status=0;check(waitpid(child,&status,0)==child&&status==0,"independent process fresh B1 then B2 continuation");
  MapDatabase db(path.string(),"resume");db.validateHistoricalRecords(.1f);
  auto links=db.loadGraphLinks();
  check(std::none_of(links.begin(),links.end(),[](const auto &l){return l.type==0&&l.to_id==9;}),"restart never bridges type-0 across domains");
  check(std::count_if(links.begin(),links.end(),[](const auto &l){return l.type==1&&l.from_id==9;})==1,"restarted domain has one attachment");
  check(std::count_if(links.begin(),links.end(),[](const auto &l){return l.type==0&&l.from_id==9&&l.to_id==10;})==1,"same restarted producer continues type-0");
  db.finish();
}

void newMapAdmission(const fs::path& path) {
  PoseGraphParameters config; config.visual.enabled = true;
  Barrier processing;
  PoseGraphBackend backend(config, {}, path.string(), {});
  std::future<void> second;
  std::future<bool> third;
  struct Cleanup {
    Barrier& barrier;
    PoseGraphBackend& backend;
    ~Cleanup() {
      barrier.release(); setHook({});
      try { backend.stopAdmission(false); backend.finish(); } catch (...) {}
    }
  } cleanup{processing, backend};
  setHook([&](const char* point, void*) { if (!std::strcmp(point, "b3-legacy-visual-insert")) processing.arrive(); });
  backend.addFrame(frame(0, anchor(0)));
  processing.wait();
  second = std::async(std::launch::async, [&] { backend.addFrame(frame(1, anchor(1))); });
  const bool independent = second.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  if (!independent) processing.release();
  second.get();
  check(independent, "new-map producer admission does not acquire worker lifecycle lock");
  clearEvents();
  third = std::async(std::launch::async, [&] {
    try { backend.addFrame(frame(2, anchor(2))); return false; }
    catch (const MapError& error) { return error.code() == MapErrorCode::Lifecycle; }
  });
  awaitEvent("legacy-admission-wait");
  const auto progress = backend.continuationProgress();
  backend.stopAdmission(false);
  const bool awakened = third.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  processing.release(); setHook({});
  check(awakened && third.get(), "stop wakes a capacity-blocked producer without accepting its tail");
  check(progress.accepted == 2 && progress.waiting == 1 && progress.high_water == 1 && progress.waiting_bytes > 0,
        "exactly one accounted waiting submap behind active work");
  backend.finish();
  check(backend.continuationProgress().completed == 2, "checked finish drains every accepted new-map submap");
}

NaviMapParameters geometryParameters() {
  NaviMapParameters p; p.enabled = false; return p;
}
void geometryProcess(const std::string &mode, const fs::path &path) {
  metric_fixture = true;
  auto nav = geometryParameters();
  const auto identity = map_config_identity(PoseGraphParameters{}, nav);
  auto changed = nav; changed.resolution *= 3; changed.hit_probability = .6;
  check(identity == map_config_identity(PoseGraphParameters{}, changed), "local navigation cannot reinterpret stored geometry");
  if (mode == "--geometry-seed") {
    MapDatabase db(path.string(), "new", identity);
    std::vector<GraphLink> links;
    std::vector<std::pair<int, Eigen::Isometry3f>> poses;
    for (int i = 0; i < 6; ++i) {
      auto f = frame(i, anchor(i), true);
      // Deliberately supply nonempty navigation: geometry storage omits it.
      db.saveSubmap(f, grid(), buildVisualScene(f)->encode());
      poses.emplace_back(i + 1, anchor(i).cast<float>());
      if (i) links.push_back({i, i + 1, 0, (anchor(i - 1).inverse() * anchor(i)).cast<float>()});
    }
    db.saveSubmapPoses(poses, links); db.finish(); return;
  }
  if (mode == "--geometry-continue") {
    PoseGraphBackend b(resumeConfig(), nav, path.string(), {});
    check(!b.latestOccupancyGrid(), "geometry reopen builds no occupancy product");
    attach(b, 6); clearEvents(); submit(b, 1); awaitEvent("b2-completed");
    check(b.hasActiveCorrection() && !b.latestOccupancyGrid(), "B1/B2 correction without navigation");
    b.finish(); return;
  }
  check(mode == "--geometry-read", "known geometry child mode");
  mapping::DescriptorArchive::IndexNodeMetadata scene_metadata;
  setenv("CUDA_VISIBLE_DEVICES", "", 1);
  std::ifstream before_stream(path, std::ios::binary);
  const std::string before{std::istreambuf_iterator<char>(before_stream), {}};
  {
    MapDatabase db(path.string(), "resume", identity);
    db.validateHistoricalRecords(.3f);
    auto *h = MapDatabaseTestAccess::handle(db);
    database_detail::Statement version(h, "PRAGMA user_version");
    check(sqlite3_step(version.get()) == SQLITE_ROW && sqlite3_column_int(version.get(), 0) == 2, "explicit geometry schema");
    database_detail::Statement tables(h, "SELECT COUNT(*) FROM sqlite_master WHERE name IN ('FlatGrid','NaviTrajectory')");
    check(sqlite3_step(tables.get()) == SQLITE_ROW && sqlite3_column_int(tables.get(), 0) == 0, "no navigation tables");
    const auto links = db.loadGraphLinks();
    check(std::count_if(links.begin(), links.end(), [](const auto &l) { return l.type == 1 && l.from_id == 7 && l.to_id == 6; }) == 1,
          "real B1 geometry constraint survives");
    check(std::count_if(links.begin(), links.end(), [](const auto &l) { return l.type == 0 && l.from_id == 7 && l.to_id == 8; }) == 1,
          "B2 same-domain constraint survives");
    check(db.loadCloud(8) && !db.loadVisualFrames(8).empty(), "geometric and visual evidence survives");
    std::size_t nodes = 0;
    db.visitHistoricalNodes([&](int id, const auto &original, const auto &, const auto &) {
      auto frames = db.loadVisualFrames(id);
      check(frames.size()==1 && frames[0].T_odom_camera() && frames[0].points()[0].geometry(), "original and appended camera evidence survives");
      check(frames[0].projection() && frames[0].projection()->width == 640 && frames[0].projection()->height == 480 &&
            frames[0].projection()->intrinsics == std::array<double,4>{300,300,320,240}, "frozen rectified projection survives three processes and continuation");
      const auto bytes = db.loadVisualScene(id);
      const auto scene = mapping::scene::FeatureMap::decode(bytes.data(),bytes.size());
      check(scene->points().size()==96 && scene->points()[0].has_position, "metric scene survives W and independent reopen");
      const auto expected = buildVisualScene(frames,original);
      check(mapping::DescriptorArchive::fingerprint(*scene)==mapping::DescriptorArchive::fingerprint(*expected),
            "persisted image evidence reconstructs exact scene fingerprint");
      scene_metadata[id] = {mapping::DescriptorArchive::fingerprint(*scene), scene->points().size()};
      ++nodes;
    });
    check(nodes==8,"metric membership includes historical, attachment and continuation nodes");
  }
  {
    PoseGraphBackend b(resumeConfig(), changed, path.string(), {});
    check(b.reconstructionDiagnostics().nodes == 8 && !b.hasActiveCorrection() && !b.latestOccupancyGrid(), "independent geometry A2");
    const auto metadata = b.historicalVisualMetadata();
    check(!metadata.empty(), "metric descriptor index restored");
    for (const auto &[id, value] : metadata) check(scene_metadata.at(id)==value,"restored index fingerprint matches archived metric scene");
    b.finish();
  }
  std::ifstream after_stream(path, std::ios::binary);
  const std::string after{std::istreambuf_iterator<char>(after_stream), {}};
  check(before == after, "geometry-only reopen is byte-preserving");
}
void geometryCycle(const fs::path &path) {
  for (const char *mode : {"--geometry-seed", "--geometry-continue", "--geometry-read"}) {
    const auto child = fork(); check(child >= 0, "fork geometry process");
    if (!child) { execl("/proc/self/exe", "sapphire_continuation_test", mode, path.c_str(), nullptr); _exit(120); }
    int status = 0; check(waitpid(child, &status, 0) == child && status == 0, std::string("independent geometry process: ") + mode);
  }
  const fs::path corrupt = path.string()+".bad-geometry";
  fs::copy_file(path,corrupt);
  sqlite3 *raw = nullptr; check(sqlite3_open(corrupt.c_str(),&raw)==SQLITE_OK,"open corrupt fixture copy");
  {
    database_detail::Statement query(raw,"SELECT payload FROM VisualScene WHERE node_id=1");
    check(sqlite3_step(query.get())==SQLITE_ROW,"read scene fixture");
    auto scene = mapping::scene::FeatureMap::decode(sqlite3_column_blob(query.get(),0),sqlite3_column_bytes(query.get(),0));
    auto points = scene->points(); points[0].position.x += .5f;
    auto bytes = mapping::scene::FeatureMap::create(0,std::move(points),mapping::scene::point_capacity)->encode();
    database_detail::Statement update(raw,"UPDATE VisualScene SET payload=? WHERE node_id=1");
    sqlite3_bind_blob(update.get(),1,bytes.data(),bytes.size(),SQLITE_TRANSIENT);
    check(sqlite3_step(update.get())==SQLITE_DONE,"write CRC-valid inconsistent geometry");
  }
  check(sqlite3_close(raw)==SQLITE_OK,"close corruption writer");
  bool rejected = false;
  try {
    MapDatabase db(corrupt.string(),"resume",map_config_identity(PoseGraphParameters{},geometryParameters()));
    db.validateHistoricalRecords(.1f);
  } catch(const MapError &e) { rejected = e.code()==MapErrorCode::HistoricalData; }
  check(rejected,"CRC-valid scene/image geometry disagreement is a hard failure");
  fs::remove(corrupt);
  std::cout << "PASS metric archive write/destroy/reopen/real B1/B2/reopen, restored index and geometry corruption rejection without navigation tables\n";
}

int main(int argc,char **argv) try {
  if (argc > 2 && std::string(argv[1]).find("--geometry-") == 0) { geometryProcess(argv[1], argv[2]); return 0; }
  if(argc>1&&std::string(argv[1])=="--restart-continue") {
    const fs::path path=argv[2];
    {
      PoseGraphBackend b(resumeConfig(),{},path.string(),{});
      check(b.reconstructionDiagnostics().nodes==8&&!b.hasActiveCorrection(),"restarted A2 has history but no live domain");
      bool denied=false;try{b.beginContinuation(77);}catch(const MapError&){denied=true;}
      check(denied&&!b.failed(),"old generation cannot establish resumed domain");
      auto a=freshAnchor();a.translation().x()+=30;
      auto result=b.attachFreshSession(frame(500,a),grid(),1,relation(),99);
      check(result.status==AttachmentStatus::Attached&&result.root_node_id==9,"fresh B1 after process restart");
      b.beginContinuation(99);clearEvents();
      // Keep this restart/domain test spatially disjoint from the earlier session.
      // Repeated identical clouds at nearby distinct poses can induce an aliased loop;
      // ordinary-loop geometry/residual/oracle has its own consistent fixture.
      auto next=a;next.translation()+=Eigen::Vector3d(100,-40,0);
      auto q=frame(501,next);auto g=grid();check(b.submitContinuation(q,g,99)==ContinuationAdmission::Accepted,"new producer B2 after restart");
      awaitEvent("b2-completed");
      auto expected=semantic(b,10);std::ofstream out(path.string()+".oracle",std::ios::binary);out.write(expected.data(),expected.size());out.close();b.finish();
    }
    oracle(path,path.string()+".oracle",10);return 0;
  }

  if(argc>1&&std::string(argv[1])=="--topology-read") {
    { MapDatabase db(argv[2],"resume");db.validateHistoricalRecords(.1f);auto *h=MapDatabaseTestAccess::handle(db);
    database_detail::Statement version(h,"PRAGMA user_version");check(sqlite3_step(version.get())==SQLITE_ROW&&sqlite3_column_int(version.get(),0)==1,"schema 1");
    auto links=db.loadGraphLinks();check(std::any_of(links.begin(),links.end(),[](auto &l){return l.from_id==101&&l.to_id==99&&l.type==1;}),"nearby cross-chain factor retained");
    // Original schema-1 reader predicate would reject this exact new edge, without reinterpreting bytes.
    check(!(101-99>3),"old global-gap validation fails closed"); }
    PoseGraphBackend backend(resumeConfig(),{},argv[2],{});
    check(backend.reconstructionDiagnostics().nodes==101&&!backend.hasActiveCorrection(),"independent A2 reconstructs nearby-ID cross-chain loop");
    backend.finish();return 0;
  }

  if(argc>1&&std::string(argv[1])=="--read") {
    PoseGraphBackend b(resumeConfig(),{},argv[2],{}); check(!b.hasActiveCorrection(),"A2 oracle has no fabricated session");
    std::ifstream in(argv[3],std::ios::binary); std::string expected{std::istreambuf_iterator<char>(in),{}};
    check(semantic(b,std::stoi(argv[4]))==expected,"incremental runtime equals independent A2"); b.finish(); return 0;
  }
  policy(); upstreamBounds();
  char temp[]="/tmp/sapphire-b2-XXXXXX"; check(mkdtemp(temp),"unique fixture directory"); const fs::path root=temp;
  if(argc>1&&std::string(argv[1])=="--geometry-cycle") {geometryCycle(root/"geometry.db");return 0;}
  if(argc>1&&std::string(argv[1])=="--new-map-finish") {newMapFinish(root/"new-finish.db");return 0;}
  if(argc>1&&std::string(argv[1])=="--benchmark") {
    for(int n:{6,84,100,512,1024})success(root/("bench-"+std::to_string(n)+".db"),n);
    for(int n:{6,84,100,512})offeredWorkload(root/("offered-"+std::to_string(n)+".db"),n);
    ordinaryLoop(root/"bench-loop.db");allKeyDelta(root/"bench-moved.db");
    grid_empty_points=8192;success(root/"bench-512-grid8192.db",512);return 0;
  }
  topology(root); visualPopulation(root); incrementalOwners(root/"owners.db");
  if(argc>1&&std::string(argv[1])=="--structural") return 0;
  newMapAdmission(root/"admission.db");
  success(root/"success.db",6); allKeyDelta(root/"delta.db"); ordinaryLoop(root/"loop.db");
  retryAndFinish(root); finishAndReset(root); workerGuard(root/"guard.db"); pipelineContinuation(root/"pipeline.db");restartContinuation(root/"restart.db");newMapFinish(root/"new-finish.db");
  for(const auto &fault:{"isam-update-entry","after-node","b2-postcommit-materialization","b2-before-visual","b2-before-occupancy","b2-runtime-ready","after-commit","unknown"}) failureCase(root/(std::string(fault)+".db"),fault);
  failureCase(root/"nonstd.db","isam-update-entry",true);
  std::cout<<"PASS B2 production continuation, failure boundaries, bounded producer and independent A2 oracle\n";
  return 0;
} catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;} catch(...){std::cerr<<"FAIL nonstandard\n";return 1;}
