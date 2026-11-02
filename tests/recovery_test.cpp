#include "pipeline/frontend.hpp"
#include "pipeline/backend.hpp"
#include "slam.hpp"
#include "database/memory.hpp"
#include <cassert>
#include <chrono>
#include <future>
#include <iostream>
#include <map>
#include <thread>
#include <unistd.h>
#include <random>
using namespace sapphire;
using namespace std::chrono_literals;
namespace sapphire {
struct SlamPipelineTestAccess {
  static void transfer(SlamPipeline& p, LidarFrame f) { assert(p.waitForCapacity()); p.transfer(std::move(f)); }
  static void verify_reset(SlamPipeline& p) {
    auto& f=*p.frontend_;
    // Only inspect while the production owner is stopped.
    p.shutdown();
    f.imu_estimator_.init_flag=true; f.imu_estimator_.has_last_imu=true;
    f.imu_estimator_.last_pcl_end_time=123; f.current_state_.g.setConstant(123);
    f.current_state_.p.setConstant(456);
    f.system_reset();
    assert(!f.imu_estimator_.init_flag && !f.imu_estimator_.has_last_imu);
    assert(f.imu_estimator_.last_pcl_end_time==0 && f.current_state_.p.isZero());
    assert(f.current_state_.g == Eigen::Vector3d(0,0,-G_m_s2));
  }
};
}
template<class F> void until(F condition) {
  const auto end=std::chrono::steady_clock::now()+5s;
  while (!condition()) { assert(std::chrono::steady_clock::now()<end); std::this_thread::sleep_for(1ms); }
}
void recovered(SlamPipeline& p, std::uint64_t generation=1) {
  until([&]{const auto s=p.recoveryProgress(); return s.generation>=generation && !s.pending && s.phase==TrackingPhase::WaitingForInput;});
  assert(!p.failed());
}
ImuMeas imu(double t) { ImuMeas m; m.timestamp=t; m.gyro.setZero();m.accel={0,0,9.81};return m; }
std::vector<LidarPoint> cloud() { std::vector<LidarPoint> p(10); for(int i=0;i<10;++i){p[i].x=2;p[i].y=i*.1;p[i].z=0;p[i].intensity=1;p[i].time_offset=i*.01;}return p; }
void synchronization() {
  Synchronizer s; MeasGroup m; RecoveryReason reason;
  assert(s.push_lidar(1,cloud(),1.1,42,7));
  assert(s.sync_packages(m).kind==SyncOutcome::Kind::Waiting);
  assert(s.push_imu(imu(1.11)));
  auto outcome=s.sync_packages(m);
  assert(outcome.kind==SyncOutcome::Kind::Invalid && outcome.reason==RecoveryReason::ImuCoverage && m.sequence==42 && m.generation==7);
  s.reset();
  for(int i=0;i<=11;++i) assert(s.push_imu(imu(.5+i*.01)));
  assert(s.push_lidar(.5,cloud(),.6,43,8));
  assert(s.sync_packages(m).kind==SyncOutcome::Kind::Ready && m.sequence==43);
  assert(!s.push_imu(imu(.1),&reason) && reason==RecoveryReason::ClockDiscontinuity);
  s.reset();
  for(double t:{1.,1.01,1.02,1.08,1.09,1.11}) assert(s.push_imu(imu(t)));
  assert(s.push_lidar(1,cloud(),1.1));
  assert(s.sync_packages(m).reason==RecoveryReason::ImuGap);
  s.reset();
  for(int i=0;i<32;++i) assert(s.push_lidar(10+i*.1,cloud()));
  assert(!s.push_lidar(14,cloud(),{},0,0,&reason) && reason==RecoveryReason::CapacityExceeded);
  assert(s.reset().size()==32);
  assert(s.push_lidar(1,cloud()));s.reset();
  s.stop_accepting();s.reset();assert(!s.push_imu(imu(2)));
}
void runtime() {
  SapphireParameters params;params.pose_graph.enabled=false;params.local_submap.thread_num=1;params.recovery.input_timeout_sec=.05;
  std::mutex mutex;std::map<std::uint64_t,ScanResult> results;
  SlamPipeline::Callbacks cb;
  cb.scan_finished=[&](const ScanResult& r){std::lock_guard<std::mutex> lock(mutex);assert(results.emplace(r.sequence,r).second);};
  SlamPipeline p(params,cb);
  const auto queued=p.push_lidar(10,cloud(),10.1);assert(queued);
  recovered(p); // No subsequent message is needed to discover total sensor silence.
  assert(p.recoveryProgress().reason==RecoveryReason::ImuSilence);
  {std::lock_guard<std::mutex>lock(mutex);assert(results.at(queued.sequence).disposition==ScanDisposition::Cancelled);}
  std::this_thread::sleep_for(150ms);assert(p.recoveryProgress().resets==1);
  // Repeated invalid attempts belong to one outage, not a storm of empty maps.
  for(int i=0;i<3;++i) {const auto rejected=p.rejectLidar(11+i,RecoveryReason::InvalidLidar);assert(!rejected);recovered(p);}
  assert(p.recoveryProgress().resets==1);
  assert(p.push_imu(imu(1))); // Cleared sensor clock baseline, even after old t=10.
  for(int i=1;i<30 && p.recoveryProgress().reason!=RecoveryReason::LidarSilence;++i) {
    std::this_thread::sleep_for(10ms);p.push_imu(imu(1+i*.01));
  }
  recovered(p);assert(p.recoveryProgress().reason==RecoveryReason::LidarSilence);
  // A queued sparse scan must produce a rejection, never a missing completion.
  assert(p.push_imu(imu(2.11)));const auto sparse=p.push_lidar(2,cloud(),2.1);assert(sparse);
  until([&]{std::lock_guard<std::mutex>lock(mutex);return results.count(sparse.sequence);});recovered(p);
  {std::lock_guard<std::mutex>lock(mutex);assert(results.at(sparse.sequence).reason==RecoveryReason::ImuCoverage);}
  for(int i=0;i<=11;++i)assert(p.push_imu(imu(3+i*.01)));
  const auto fresh=p.push_lidar(3,cloud(),3.1);assert(fresh);
  until([&]{std::lock_guard<std::mutex>lock(mutex);return results.count(fresh.sequence);});
  {std::lock_guard<std::mutex>lock(mutex);assert(results.at(fresh.sequence).disposition==ScanDisposition::Initializing);}
  SlamPipelineTestAccess::verify_reset(p);
  // Shutdown acknowledges input that could never be synchronized.
  params.recovery.input_timeout_sec=0;
  { std::lock_guard<std::mutex>lock(mutex);results.clear(); }
  SlamPipeline tail(params,cb);const auto last=tail.push_lidar(4,cloud(),4.1);assert(last);
  // Sequence numbers are local to each pipeline, use a different sink below.
  tail.shutdown();
}
void barrier_and_shutdown() {
  SapphireParameters p;p.pose_graph.enabled=false;p.recovery.input_timeout_sec=0;p.local_submap.thread_num=1;
  std::promise<void> entered, release;auto released=release.get_future().share();std::atomic_bool armed{false};
  SlamPipeline::Callbacks cb;
  cb.owner_reset=[&](bool begin){if(begin && armed.exchange(false)){entered.set_value();released.wait();}};
  SlamPipeline pipeline(p,cb);armed=true;pipeline.notifyInputLoss();
  assert(entered.get_future().wait_for(2s)==std::future_status::ready);
  assert(pipeline.recoveryProgress().pending && !pipeline.push_imu(imu(1)));
  assert(!pipeline.push_lidar(1,cloud(),1.1));
  auto stopped=std::async(std::launch::async,[&]{pipeline.shutdown();});
  release.set_value();assert(stopped.wait_for(2s)==std::future_status::ready);stopped.get();
  assert(pipeline.recoveryProgress().phase==TrackingPhase::Stopped && !pipeline.failed());
  // A required output fence failure cannot be swallowed and resume publication.
  cb.owner_reset=[&](bool begin){if(begin && armed.exchange(false))throw std::runtime_error("fence failed");};
  SlamPipeline failing(p,cb);armed=true;failing.notifyInputLoss();until([&]{return failing.failed();});
  failing.shutdown();assert(failing.recoveryProgress().phase==TrackingPhase::Failed);
}
LidarFrame frame(double t,double journey) {
  LidarFrame f(StateGroup{},GaussianCloud{},t,journey);
  for(int i=0;i<400;++i){GaussianPoint q;q.mean={std::sin(i*.3f)*3,std::cos(i*.7f)*2,std::sin(i*.17f)};q.covariance=Eigen::Vector3f(.01,.02,.03).asDiagonal();q.N=20;q.radius=.5;q.voxel_key={i,0,0,0};f.gaussians.push_back(q);}
  return f;
}
void asynchronous_reset() {
  SapphireParameters p;p.local_submap.thread_num=1;p.recovery.input_timeout_sec=0;
  p.pose_graph.enabled=true;p.pose_graph.submap_travel_distance=1;
  p.pose_graph.mapping.relocalizer.spatial_enabled=false;p.pose_graph.mapping.relocalizer.global_enabled=false;
  const auto directory=std::filesystem::temp_directory_path()/("sapphire-async-reset-"+std::to_string(::getpid()));
  std::filesystem::remove_all(directory);p.pose_graph.database_path=(directory/"map.db").string();
  std::promise<void> entered,release;auto released=release.get_future().share();
  std::atomic_bool armed{true},finished{false};std::thread::id backend_thread,frontend_thread;
  SlamPipeline::Callbacks cb;
  cb.ready_changed=[&] {if(armed.exchange(false)){backend_thread=std::this_thread::get_id();entered.set_value();released.wait();}};
  cb.scan_finished=[&](const ScanResult& result){if(result.generation==1){frontend_thread=std::this_thread::get_id();finished=true;}};
  {
    SlamPipeline pipeline(p,cb);
    SlamPipelineTestAccess::transfer(pipeline,frame(10,0));SlamPipelineTestAccess::transfer(pipeline,frame(11,2));
    assert(entered.get_future().wait_for(5s)==std::future_status::ready);
    pipeline.notifyInputLoss();recovered(pipeline);
    double stamp;Eigen::Isometry3d correction,pose;
    assert(!pipeline.captureReady(true,stamp,correction,pose));
    for(int i=0;i<=11;++i)assert(pipeline.push_imu(imu(1+i*.01)));
    assert(pipeline.push_lidar(1,cloud(),1.1));until([&]{return finished.load();});
    assert(frontend_thread!=backend_thread);
    release.set_value();pipeline.drain();pipeline.close();
  }
  std::filesystem::remove_all(directory);
}
void idle_relocalization_publication() {
  SapphireParameters p;p.local_submap.thread_num=1;p.recovery.input_timeout_sec=0;
  p.pose_graph.enabled=true;p.pose_graph.submap_travel_distance=1;
  auto& mapping=p.pose_graph.mapping;
  mapping.relocalizer.global_enabled=true;mapping.relocalizer.spatial_enabled=true;
  mapping.relocalizer.spatial_max_graph_hops=0;mapping.relocalizer.path_max_nodes=1;
  mapping.memory.recent_nodes=1;mapping.registration.lidar_bbs_enabled=false;
  const auto directory=std::filesystem::temp_directory_path()/("sapphire-idle-reloc-"+std::to_string(::getpid()));
  std::filesystem::remove_all(directory);p.pose_graph.database_path=(directory/"map.db").string();
  const auto sample=[](double stamp,double journey) {
    auto f=frame(stamp,journey);f.gaussians.clear();
    std::mt19937 engine(19);std::uniform_real_distribution<float> uniform(-1,1);
    for(int i=0;i<1000;++i) {
      GaussianPoint q;const int group=i%3;
      q.mean={uniform(engine)*(group==0?7:2)+group*3,uniform(engine)*(group==1?6:1)-group*2,uniform(engine)*(group==2?3:1)+group};
      q.covariance=Eigen::Vector3f(.008f,.02f,.04f).asDiagonal();q.N=17;q.radius=.6;q.voxel_key={i,-i,i+2,i%3};q.is_plane=true;
      f.gaussians.push_back(q);
    }
    return f;
  };
  std::atomic_uint publications{0};SlamPipeline::Callbacks cb;cb.ready_changed=[&]{++publications;};
  {
    SlamPipeline pipeline(p,cb);
    SlamPipelineTestAccess::transfer(pipeline,sample(10,0));SlamPipelineTestAccess::transfer(pipeline,sample(11,2));
    until([&]{return pipeline.continuationProgress().completed==1;});
    pipeline.notifyInputLoss();recovered(pipeline);
    SlamPipelineTestAccess::transfer(pipeline,sample(1,0));SlamPipelineTestAccess::transfer(pipeline,sample(2,2));
    // Keep the pipeline alive without new input or drain. The geometry worker
    // must publish its committed solution via the idle backend owner.
    until([&]{auto state=pipeline.continuationProgress();return state.generation==1 && state.completed==1 && state.completed_revision && state.ready_revision>*state.completed_revision;});
    assert(publications.load()>=3);
    double stamp;Eigen::Isometry3d correction,pose;
    const auto ready=pipeline.captureReady(false,stamp,correction,pose);
    assert(ready && ready->generation==1 && stamp==1 && (correction.matrix()-Eigen::Matrix4d::Identity()).norm()<.02);
    pipeline.drain();pipeline.close();
  }
  std::filesystem::remove_all(directory);
}
void persistence(const std::string& config,const std::filesystem::path& dir) {
  auto p=load_parameters(config,load_sensor_parameters(std::filesystem::path(config).parent_path()/"sensors.toml").lidars.front());
  p.local_submap.thread_num=1;p.recovery.input_timeout_sec=0;p.pose_graph.enabled=true;p.pose_graph.submap_travel_distance=1;
  p.pose_graph.mapping.registration.lidar_bbs_enabled=false;p.pose_graph.mapping.relocalizer.spatial_enabled=false;p.pose_graph.mapping.relocalizer.global_enabled=false;
  p.pose_graph.database_path=(dir/"original.db").string();
  {
    SlamPipeline pipeline(p);
    SlamPipelineTestAccess::transfer(pipeline,frame(10,0));SlamPipelineTestAccess::transfer(pipeline,frame(11,2));
    until([&]{return pipeline.continuationProgress().completed==1;});
    pipeline.notifyInputLoss();recovered(pipeline);
    assert(!pipeline.continuationProgress().ready_available);
    // This low timestamp and origin must be admitted only into the new independent segment.
    SlamPipelineTestAccess::transfer(pipeline,frame(1,0));SlamPipelineTestAccess::transfer(pipeline,frame(2,2));
    until([&]{return pipeline.continuationProgress().completed==1;});
    pipeline.drain();pipeline.close();
  }
  // Resume admits an independent segment automatically; recovery seals it intact.
  p.pose_graph.map_mode="resume";
  SlamPipeline resumed(p);
  SlamPipelineTestAccess::transfer(resumed,frame(20,0));SlamPipelineTestAccess::transfer(resumed,frame(21,2));
  until([&]{return resumed.continuationProgress().completed==1;});
  resumed.notifyInputLoss();recovered(resumed);
  SlamPipelineTestAccess::transfer(resumed,frame(3,0));SlamPipelineTestAccess::transfer(resumed,frame(4,2));
  until([&]{return resumed.continuationProgress().completed==1;});
  resumed.drain();resumed.close();
}
int main(int argc,char**argv) {
  if(argc==4) {
    auto p=load_parameters(argv[1],load_sensor_parameters(std::filesystem::path(argv[1]).parent_path()/"sensors.toml").lidars.front());
    p.pose_graph.mapping.relocalizer.global_enabled=false;
    Mapping map;map.init(p.pose_graph.mapping,argv[2]);
    const double stamps[]{10,1,20,3};
    for (int id=1;id<=4;++id) {
      const auto node=map.getNodeCopy(id);assert(node.lidar_cloud() && node.lidar_cloud()->size()==400);
      assert(node.getStamp()==stamps[id-1] && node.mapId()==id);
      assert(map.getMemory()->metric_component(id)==std::set<int>{id});
    }
    map.triggerNewMap();
    assert(map.process_node(std::make_unique<Node>(std::make_shared<GaussianCloud>(frame(30,0).gaussians),30,SE3f::identity()),Mat66_t<double>::Identity()));
    assert(map.getLastLocationId()==5);
    map.checkpoint();map.close();
  } else if(argc==3) persistence(argv[1],argv[2]);else {synchronization();runtime();barrier_and_shutdown();asynchronous_reset();idle_relocalization_publication();}
  std::cout<<"PASS recovery lifecycle\n";
}
