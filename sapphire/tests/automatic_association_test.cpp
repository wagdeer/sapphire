#include "backend/graph/pose_graph.hpp"
#include "pipeline.hpp"
#include <condition_variable>
#include <future>
#include <thread>
#include "backend/storage/map_database.hpp"
#include "backend/storage/memory.hpp"
#include "backend/visual/visual_loop.hpp"
#include "backend/visual/feature/visual_tracker.hpp"
#include "backend/registration/loop_closure.hpp"
#include <sys/wait.h>
#include <unistd.h>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace sapphire;
namespace fs = std::filesystem;
namespace sapphire {
struct SlamPipelineTestAccess {
  static void submit(SlamPipeline &p,SubmapFrame q) { p.submitAutomaticSubmap(std::move(q),LocalGrid(.1)); }
  static void marginal(SlamPipeline &p,MargiFrame q) {
    std::lock_guard<std::mutex> lock(p.marginal_mutex_);p.marginal_frames_.push_back(std::move(q));p.marginal_cv_.notify_all();
  }
};
}
namespace {
std::mutex barrier_mutex;std::condition_variable barrier_cv;
bool block_score=false,score_entered=false;
std::atomic<int> bbs{0}, scores{0}, refinements{0}, force_rejection{0};
std::atomic<bool> uncertain_refresh_commit{false};
void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
PoseGraphParameters config(bool refresh=false) {
  PoseGraphParameters p; p.scene_refresh=refresh; p.map_mode="resume"; p.visual.enabled=true; p.visual.top_k=5; p.visual.min_matches=20;
  p.visual.left.width=640;p.visual.left.height=480;p.visual.left.intrinsics={300,310,320,240};return p;
}
NaviMapParameters navigation() { NaviMapParameters p;p.enabled=false;return p; }
Eigen::Isometry3d historical() {
  auto p=Eigen::Isometry3d::Identity();p.linear()=Eigen::AngleAxisd(.18,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation()=Eigen::Vector3d(12,-5,2);return p;
}
Eigen::Isometry3d relative(double advance=0) {
  auto p=Eigen::Isometry3d::Identity();
  p.linear()=(Eigen::AngleAxisd(.13,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-.08,Eigen::Vector3d::UnitX())).toRotationMatrix();
  p.translation()=Eigen::Vector3d(.4+advance,-.2,.1);return p;
}
Eigen::Isometry3d fresh() {
  auto p=Eigen::Isometry3d::Identity();p.linear()=Eigen::AngleAxisd(-.7,Eigen::Vector3d::UnitY()).toRotationMatrix();
  p.translation()=Eigen::Vector3d(-90,25,4);return p;
}
struct Fixture {
  cv::Mat descriptors;std::vector<VisualPoint> target_pixels;
  std::vector<Eigen::Vector3d> landmarks;GaussianCloud geometry;
  Eigen::Isometry3d target_camera=Eigen::Isometry3d::Identity(),query_camera=Eigen::Isometry3d::Identity();
  Fixture() {
    target_camera.linear()=Eigen::AngleAxisd(.07,Eigen::Vector3d::UnitX()).toRotationMatrix();target_camera.translation()=Eigen::Vector3d(.2,-.1,.05);
    query_camera.linear()=Eigen::AngleAxisd(-.04,Eigen::Vector3d::UnitY()).toRotationMatrix();query_camera.translation()=Eigen::Vector3d(.3,.1,.08);
    cv::RNG rng(93077);descriptors=cv::Mat(120,32,CV_8UC1);rng.fill(descriptors,cv::RNG::UNIFORM,0,256);
    for(int i=0;i<120;++i) {
      double u=100+(i%12)*36,v=90+(i/12)*31,z=rng.uniform(4.,7.);
      Eigen::Vector3d pc((u-320)*z/300,(v-240)*z/310,z);landmarks.push_back(target_camera*pc);
      VisualPoint point({float(u),float(v)},0,1);std::array<VisualRayObservation,3> rays;
      for(int j=0;j<3;++j) {
        auto pose=target_camera;pose.translation()+=Eigen::Vector3d((j-2)*.2,.02*(j-2),0);
        Eigen::Vector3d ray=pose.inverse()*landmarks.back();rays[j]={.1*j,pose,ray.head<2>()/ray.z()};
      }
      auto depth=triangulateVisualTrack(rays,300,310,{});check(bool(depth),"visual-only triangulation");point.set_tracking(i+1,depth);target_pixels.push_back(point);
    }
    // Independent synthetic Gaussian samples; never assigned to image features.
    for(int i=0;i<400;++i) {
      GaussianPoint p;p.N=20;p.mean={rng.uniform(-3.f,3.f),rng.uniform(-3.f,3.f),rng.uniform(-1.f,2.f)};
      p.covariance=Eigen::Vector3f(.001,.01,.04).asDiagonal();p.regularize();geometry.push_back(p);
    }
  }
  SubmapFrame frame(std::uint64_t id,const Eigen::Isometry3d &odom,bool target=false,bool projection=true,
                    bool bad_cloud=false,double advance=0,bool visual=true,bool padded=false,bool metric=false) const {
    const auto transform=relative(advance);
    auto cloud=std::make_shared<GaussianCloud>(geometry);
    if(padded)cloud->reserve(2*1024*1024/sizeof(GaussianPoint));
    if(!target) for(auto &p:*cloud) {
      p.mean=(transform.inverse()*p.mean.cast<double>()).cast<float>();
      p.covariance=(transform.linear().transpose()*p.covariance.cast<double>()*transform.linear()).cast<float>();
      if(bad_cloud)p.mean.x()+=30;
    }
    cpu::VoxelMaps voxels;voxels.set_min_res(.25);voxels.create_voxelmaps(cloud->size(),[&](std::size_t i){return (*cloud)[i].mean;});
    VisualFrame image(double(id),0);
    if(visual) {
      std::vector<VisualPoint> points=target_pixels;
      if(!target) {points.clear();for(const auto &xyz:landmarks) {
        Eigen::Vector3d q=query_camera.inverse()*transform.inverse()*xyz;
        points.emplace_back(Eigen::Vector2f(float(300*q.x()/q.z()+320),float(310*q.y()/q.z()+240)),0,1);
        if(metric) {
          std::array<VisualRayObservation,3> rays;
          for(int j=0;j<3;++j) {
            auto pose=query_camera;pose.translation()+=Eigen::Vector3d((j-2)*.2,.02*(j-2),0);
            Eigen::Vector3d ray=pose.inverse()*transform.inverse()*xyz;
            rays[j]={.1*j,pose,ray.head<2>()/ray.z()};
          }
          auto depth=triangulateVisualTrack(rays,300,310,{});check(bool(depth),"query visual-only depth");
          points.back().set_tracking(points.size(),depth);
        }
      }}
      image.update_features(std::move(points),descriptors.clone());image.set_observation_pose(odom*(target?target_camera:query_camera));
      if(projection)image.set_projection({640,480,{300,310,320,240}});
    }
    LioFrame lio;lio.pcd=cloud;lio.T_odom_base=odom;lio.timestamp=double(id);
    std::vector<VisualFrame> images;images.push_back(std::move(image));
    return SubmapFrame(id,std::move(lio),id,id,1,0,0,voxels.release_data(),{{double(id),odom}},NavigationPath{},std::move(images));
  }
};
std::string bytes(const fs::path &p) {std::ifstream in(p,std::ios::binary);return {std::istreambuf_iterator<char>(in),{}};}
void seed(const fs::path &path,int count=1,int conflict=0,bool refresh=false) {
  Fixture f;MapDatabase db(path.string(),"new",map_config_identity(config(refresh),navigation()));
  std::vector<std::pair<int,Eigen::Isometry3f>> poses;std::vector<GraphLink> links;
  for(int i=0;i<count;++i) {
    auto pose=historical();if(conflict==1)pose.translation().x()+=i*25;
    if(conflict==2)pose.linear()=pose.linear()*Eigen::AngleAxisd(i*.5,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto q=f.frame(i,pose,true);db.saveSubmap(q,LocalGrid(.1),buildVisualScene(q)->encode());poses.emplace_back(i+1,pose.cast<float>());
    if(i)links.push_back({i,i+1,0,(poses[i-1].second.inverse()*poses[i].second)});
  }
  db.saveSubmapPoses(poses,links);db.finish();
}
void rejection(const fs::path &path,int kind) {
  seed(path,(kind==1||kind==7)?2:1,kind==7?2:int(kind==1));const auto before=bytes(path);Fixture fixture;
  {
    PoseGraphBackend backend(config(),navigation(),path.string(),{});
    auto q=fixture.frame(900,fresh(),false,kind!=2,kind==3,0,kind!=4);
    const auto revision=backend.graphRevision();bbs=scores=refinements=0;force_rejection=kind;
    auto result=backend.attachFreshSessionAutomatically(q,LocalGrid(.1),77);
    check(result.status==((kind==1||kind==7)?AttachmentStatus::AmbiguousAssociation:AttachmentStatus::NoAssociation),"ambiguity/missing/geometric evidence leaves session unattached");
    check(q.id()==900 && !backend.failed() && !backend.hasActiveCorrection() && backend.graphRevision()==revision,"clean association rejection retains query/history");
    check(bbs==0,"no BBS fallback after visual proposal/rejection");
    if(kind==1||kind==7)check(scores==2&&refinements==2,"all competing places checked before commit");
    if(kind==3)check(scores==1&&refinements==0,"wrong geometry rejected before GICP");
    check(bytes(path)==before,"rejected association does not change DB bytes");
    force_rejection=0;
    if(kind!=1&&kind!=7) {
      auto next=fixture.frame(901,fresh());
      check(backend.attachFreshSessionAutomatically(next,LocalGrid(.1),77).status==AttachmentStatus::Attached,"subsequent evidence may retry on same clean backend");
    }
    backend.finish();
  }
}
void attach(const fs::path &path,bool continue_session,int historical_count=1) {
  Fixture fixture;int notifications=0;
  PoseGraphBackend backend(config(),navigation(),path.string(),{},{},[&]{++notifications;});
  auto q=fixture.frame(900,fresh());bbs=scores=refinements=0;
  const auto result=backend.attachFreshSessionAutomatically(q,LocalGrid(.1),77);
  check(result.status==AttachmentStatus::Attached && result.root_node_id==historical_count+1 && q.id()==std::uint64_t(historical_count),"automatic newest-history target and persistent root identity");
  check(bbs==0 && scores==historical_count && refinements==historical_count,"one visual verification per candidate, no repeated registration during B1");
  check(notifications==1 && backend.hasActiveCorrection(),"ready notification without next input");
  check((result.correction->matrix()-(historical()*relative()*fresh().inverse()).matrix()).norm()<.02,"unrelated odometry origin maps through asymmetric metric relation");
  check(backend.continuationProgress().root_source==900,"producer source is not overwritten by DB identity");
  if(continue_session) {
    backend.beginContinuation(77);
    auto q1=fixture.frame(901,fresh()*relative().inverse()*relative(.15),false,true,false,.15,false);
    LocalGrid grid(.1);check(backend.submitContinuation(q1,grid,77)==ContinuationAdmission::Accepted,"automatic root unlocks same-producer B2");
  }
  backend.drain();
  if(continue_session)check(backend.continuationProgress().completed==1,"B2 accepted work drains");
  backend.close();
}
void reopen(const fs::path &path) {
  MapDatabase db(path.string(),"resume",map_config_identity(config(),navigation()));db.validateHistoricalRecords(.1);
  auto links=db.loadGraphLinks();
  auto loop=std::find_if(links.begin(),links.end(),[](const auto &l){return l.from_id==2&&l.to_id==1&&l.type==1;});
  check(loop!=links.end()&&(loop->transform.cast<double>().matrix()-relative().inverse().matrix()).norm()<.02,"auto factor direction survives independent process");
  check(std::any_of(links.begin(),links.end(),[](const auto &l){return l.from_id==2&&l.to_id==3&&l.type==0;}),"same-session odometry link survives");
  check(db.loadVisualFrames(2).at(0).projection().has_value(),"auto-attached visual model survives");db.finish();
  sqlite3 *raw=nullptr;check(sqlite3_open_v2(path.c_str(),&raw,SQLITE_OPEN_READONLY,nullptr)==SQLITE_OK,"read-only archive inspection");
  {database_detail::Statement tables(raw,"SELECT count(*) FROM sqlite_master WHERE name IN ('FlatGrid','NaviTrajectory')");
   check(sqlite3_step(tables.get())==SQLITE_ROW&&sqlite3_column_int(tables.get(),0)==0,"no persisted navigation tables");}
  check(sqlite3_close(raw)==SQLITE_OK,"inspection close");
  PoseGraphBackend backend(config(),navigation(),path.string(),{});
  check(backend.reconstructionDiagnostics().nodes==3&&!backend.hasActiveCorrection()&&!backend.latestOccupancyGrid(),"independent A2 has committed scene graph and no invented live session/navigation");
  check(!backend.historicalVisualMetadata().empty(),"historical descriptor index rebuild");backend.finish();
}
SapphireParameters pipelineConfig(const fs::path &path) {
  SapphireParameters p;p.general.save_map=0;p.pose_graph=config();p.navi_map=navigation();
  p.pose_graph.database_path=path.string();p.pose_graph.automatic_attachment=true;
  p.pose_graph.visual.tracking_enabled=true;
  p.pose_graph.visual.left.camera_to_imu_rotation={1,0,0,0,1,0,0,0,1};
  p.pose_graph.visual.left.camera_to_imu_translation={0,0,0};
  return p;
}
template<class F>void await(F done) {
  auto end=std::chrono::steady_clock::now()+std::chrono::seconds(10);
  while(!done()) {check(std::chrono::steady_clock::now()<end,"bounded asynchronous completion");std::this_thread::sleep_for(std::chrono::milliseconds(5));}
}
void pipelineAttach(const fs::path &path) {
  Fixture f;auto p=pipelineConfig(path);std::atomic<int> ready{0};OutputSink sink;sink.ready_changed=[&]{++ready;};
  SlamPipeline pipeline(p,sink);
  // Root has geometry but no usable image; a later view initializes that exact root.
  SlamPipelineTestAccess::submit(pipeline,f.frame(0,fresh(),false,true,false,0,false));
  await([&]{return pipeline.continuationProgress().association_attempts==1;});
  const auto pending=pipeline.continuationProgress();
  check(pending.association_pending==1&&!pending.automatically_attached&&!pending.root_source,
        "unrecognized root retained without inventing a live attachment; historical revision may already be ready");
  SlamPipelineTestAccess::submit(pipeline,f.frame(1,fresh()*relative().inverse()*relative(.15),false,true,false,.15));
  // Stop immediately: accepted stage work must attach and drain without another input.
  const auto result=pipeline.drain();
  check(!pipeline.failed()&&result.automatically_attached&&result.association_pending==0&&result.association_bytes==0,
        "automatic startup queue drained");
  check(result.root_source==0&&result.last_completed==1&&result.completed==1&&result.accepted==1&&ready>0,
        "retained root and later observation keep producer order through B1/B2 and final ready");
  pipeline.close();
}
void pipelineRead(const fs::path &path) {
  MapDatabase db(path.string(),"resume",map_config_identity(config(),navigation()));db.validateHistoricalRecords(.1);
  const auto links=db.loadGraphLinks();
  auto root=std::find_if(links.begin(),links.end(),[](const auto &l){return l.from_id==2&&l.to_id==1&&l.type==1;});
  check(root!=links.end()&&(root->transform.cast<double>().matrix()-relative().inverse().matrix()).norm()<.02,
        "later view initializes independently verified original root, not later submap's factor");
  check(!db.loadVisualFrames(2).at(0).has_features()&&db.loadVisualFrames(3).at(0).projection(),
        "root images never replaced by later observations and backlog evidence survives");
  check(std::any_of(links.begin(),links.end(),[](const auto &l){return l.from_id==2&&l.to_id==3&&l.type==0;}),"original-order B2 relation persisted");db.finish();
  PoseGraphBackend b(config(),navigation(),path.string(),{});check(b.reconstructionDiagnostics().nodes==3&&!b.hasActiveCorrection(),"independent automatic frontend reopen");b.finish();
}
void frontendCases(const fs::path &root) {
  Fixture f;
  {
    auto p=pipelineConfig(root/"config.db");const auto identity=map_config_identity(p.pose_graph,p.navi_map);validate_parameters(p);
    p.pose_graph.automatic_attachment=false;p.pose_graph.association_pending_submaps=16;
    check(identity==map_config_identity(p.pose_graph,p.navi_map),"automatic policy is not persistent scene identity");
    p.pose_graph.automatic_attachment=true;p.pose_graph.visual.tracking_enabled=false;bool rejected=false;
    try{validate_parameters(p);}catch(const std::invalid_argument&){rejected=true;}check(rejected,"automatic mode requires visual tracking");
    const auto cfg=root/"auto.toml";std::ofstream out(cfg);
    out<<"[map]\nmode='resume'\ndatabase_path='unused.db'\nautomatic_attachment=true\nassociation_pending_submaps=7\nassociation_pending_mb=64\n"
         "[visual_loop]\nenabled=true\ntracking_enabled=true\n[visual_loop.left]\nwidth=640\nheight=480\nintrinsics=[300,310,320,240]\n"
         "camera_to_imu_rotation=[1,0,0,0,1,0,0,0,1]\ncamera_to_imu_translation=[0,0,0]\n";out.close();
    const auto loaded=load_parameters(cfg);check(loaded.pose_graph.automatic_attachment&&loaded.pose_graph.association_pending_submaps==7&&loaded.pose_graph.association_pending_mb==64,"TOML automatic policy reaches core/ROS loader");
  }
  for(bool budget:{false,true}) {
    const auto path=root/(budget?"budget.db":"unresolved.db");seed(path);const auto before=bytes(path);
    auto p=pipelineConfig(path);p.pose_graph.association_pending_submaps=2;SlamPipeline pipeline(p);
    SlamPipelineTestAccess::submit(pipeline,f.frame(0,fresh(),false,true,false,0,false));
    await([&]{return pipeline.continuationProgress().association_attempts==1;});
    if(budget) {
      SlamPipelineTestAccess::submit(pipeline,f.frame(1,fresh(),false,true,false,0,false));
      await([&]{return pipeline.continuationProgress().association_attempts==2;});bool rejected=false;
      try{SlamPipelineTestAccess::submit(pipeline,f.frame(2,fresh()));}catch(const std::length_error&){rejected=true;}
      check(rejected&&pipeline.continuationProgress().association_pending==2&&pipeline.continuationProgress().association_high_water==2,"pending cap rejects overflow without discarding retained observations");
    }
    bool unresolved=false;try{(void)pipeline.drain();}catch(const MapError &e){unresolved=e.code()==MapErrorCode::ResumeUnavailable;}
    check(unresolved&&!pipeline.failed()&&bytes(path)==before,"checked unresolved shutdown cannot report durable success or mutate history");pipeline.close();
  }
  {
    const auto path=root/"byte-bound.db";seed(path);auto parameters=pipelineConfig(path);parameters.pose_graph.association_pending_mb=1;
    SlamPipeline pipeline(parameters);auto q=f.frame(0,fresh(),false,true,false,0,true,true);
    bool rejected=false;try{SlamPipelineTestAccess::submit(pipeline,std::move(q));}catch(const std::length_error&){rejected=true;}
    check(rejected&&pipeline.continuationProgress().association_pending==0&&pipeline.unacceptedTailDrops()==1,"byte budget counts retained capacity before ownership admission");
    pipeline.drain();pipeline.close();
  }
  {
    const auto path=root/"mapping-tail.db";seed(path);SlamPipeline pipeline(pipelineConfig(path));
    StateGroup state;state.R=fresh().linear();state.p=fresh().translation();GaussianCloud cloud=f.geometry;
    for(auto &point:cloud) {point.mean=(fresh()*point.mean.cast<double>()).cast<float>();
      point.covariance=(fresh().linear()*point.covariance.cast<double>()*fresh().linear().transpose()).cast<float>();}
    SlamPipelineTestAccess::marginal(pipeline,MargiFrame(state,std::move(cloud),1,Eigen::Matrix<double,6,1>::Ones(),0));
    bool unresolved=false;try{(void)pipeline.drain();}catch(const MapError &e){unresolved=e.code()==MapErrorCode::ResumeUnavailable;}
    check(unresolved&&pipeline.continuationProgress().association_pending==1&&pipeline.unacceptedTailDrops()==0,
          "real mapping builder flushes automatic short tail into association stage on shutdown");pipeline.close();
  }
  {
    const auto path=root/"cancel.db";seed(path);const auto before=bytes(path);SlamPipeline pipeline(pipelineConfig(path));
    {std::lock_guard<std::mutex> lock(barrier_mutex);block_score=true;score_entered=false;}
    SlamPipelineTestAccess::submit(pipeline,f.frame(0,fresh()));
    {std::unique_lock<std::mutex> lock(barrier_mutex);check(barrier_cv.wait_for(lock,std::chrono::seconds(10),[]{return score_entered;}),"verifier entered barrier");}
    // Verifier holds backend lifecycle, but producer still admits bounded work.
    SlamPipelineTestAccess::submit(pipeline,f.frame(1,fresh()*relative().inverse()*relative(.15),false,true,false,.15));
    check(pipeline.continuationProgress().association_pending==2,"slow verification does not hold producer queue mutex");
    pipeline.invalidateOdometryDomain();
    {std::lock_guard<std::mutex> lock(barrier_mutex);block_score=false;}barrier_cv.notify_all();
    auto stopped=std::async(std::launch::async,[&]{pipeline.shutdown();});check(stopped.wait_for(std::chrono::seconds(10))==std::future_status::ready,"cancellation wakes all stage waits");stopped.get();
    bool failed=false;try{(void)pipeline.drain();}catch(...){failed=true;}
    check(failed&&pipeline.failed()&&bytes(path)==before,"canceled verifier cannot attach the old odometry domain");pipeline.close();
  }
  {
    const auto path=root/"retryable-stop.db";seed(path);SlamPipeline pipeline(pipelineConfig(path));
    force_rejection=8;SlamPipelineTestAccess::submit(pipeline,f.frame(0,fresh()));
    await([&]{return pipeline.continuationProgress().automatically_attached;});
    for(int i=1;i<=3;++i)SlamPipelineTestAccess::submit(pipeline,f.frame(i,fresh()*relative().inverse()*relative(i*.15),false,true,false,i*.15,false));
    await([&]{return pipeline.continuationProgress().head==ContinuationHead::Retryable;});
    auto drained=std::async(std::launch::async,[&]{try{(void)pipeline.drain();return false;}catch(...){return true;}});
    check(drained.wait_for(std::chrono::seconds(10))==std::future_status::ready&&drained.get(),"retryable B2 head cannot deadlock automatic shutdown");
    force_rejection=0;pipeline.close();
  }
  {
    const auto path=root/"numeric-frontend.db";seed(path);const auto before=bytes(path);SlamPipeline pipeline(pipelineConfig(path));
    force_rejection=6;SlamPipelineTestAccess::submit(pipeline,f.frame(0,fresh()));await([&]{return pipeline.failed();});
    check(bool(pipeline.producerFailure())&&bytes(path)==before,"association worker numerical error reaches producer failure");force_rejection=0;
    bool failed=false;try{(void)pipeline.drain();}catch(...){failed=true;}check(failed,"checked drain reports worker failure");pipeline.close();
  }
}
std::int64_t scalar(const fs::path &path,const char *sql) {
  sqlite3 *db=nullptr;check(sqlite3_open_v2(path.c_str(),&db,SQLITE_OPEN_READONLY,nullptr)==SQLITE_OK,"read refresh archive");
  std::int64_t result;
  {database_detail::Statement q(db,sql);check(sqlite3_step(q.get())==SQLITE_ROW,"refresh scalar query");result=sqlite3_column_int64(q.get(),0);}
  check(sqlite3_close(db)==SQLITE_OK,"close refresh inspector");return result;
}
void refreshCycle(const fs::path &path) {
  Fixture f;const auto before=scalar(path,"SELECT count(*) FROM Node");
  PoseGraphBackend b(config(true),navigation(),path.string(),{});
  auto q=f.frame(900,fresh(),false,true,false,0,true,false,true);
  const auto result=b.attachFreshSessionAutomatically(q,LocalGrid(.1),77);
  check(result.status==AttachmentStatus::Attached&&result.root_node_id==before+1,"automatic refresh root");
  check(b.historicalVisualMetadata().size()==1,"only newest retained scene searchable after refresh");
  for(std::uint64_t key=0;key<std::uint64_t(before);++key)
    check(b.historicalSpatialCandidates(key).size()<=1,"retired spatial targets absent across independent attachment");
  check(b.continuationProgress().ready_revision==b.graphRevision(),"ready includes scene-only revision");
  b.finish();
  check(scalar(path,"SELECT count(*) FROM LaserRecord")==1&&scalar(path,"SELECT count(*) FROM VisualScene")==1,
        "covered revisit actually replaces durable heavy payload");
}
void refreshRead(const fs::path &path) {
  MapDatabase db(path.string(),"resume",map_config_identity(config(true),navigation()));db.validateHistoricalRecords(.1);
  const auto n=scalar(path,"SELECT count(*) FROM Node");
  check(n>1&&db.sceneActive(n)&&!db.sceneActive(1),"retirement survives independent reopen");
  check(db.loadGraphLinks().size()==std::size_t(n-1),"all graph identities and historical factors retained");db.finish();
  check(scalar(path,"PRAGMA user_version")==3,"explicit retained-scene schema");
  bool rejected=false;try{MapDatabase legacy(path.string(),"resume",map_config_identity(config(),navigation()));}catch(const MapError&){rejected=true;}
  check(rejected,"legacy identity cannot silently interpret retired payloads");
  PoseGraphBackend b(config(true),navigation(),path.string(),{});
  check(b.reconstructionDiagnostics().nodes==std::size_t(n)&&!b.hasActiveCorrection(),"all graph nodes reconstruct without retired geometry");
  check(b.historicalSpatialCandidates(n-1).empty(),"cold newest scene has no retired spatial targets");
  b.finish();
}
void directRefreshPair(const fs::path &path,int kind) {
  Fixture f;Memory memory(path.string(),{}, {},map_config_identity(config(true),navigation()),"new");
  auto old=f.frame(0,historical(),true);
  if(kind==0) f.geometry.front().mean.z()+=.02f; // fresh observed geometry, not byte-identical deduplication
  auto current=f.frame(1,fresh(),false,true,kind==3,0,true,false,kind!=1);
  if(kind==2) {
    auto &image=current.visual_frames()[0];auto descriptors=image.descriptors().clone();descriptors.setTo(0);
    image.update_features(image.points(),descriptors);
  }
  auto stored_old=buildVisualScene(old);
  if(kind==6) {
    auto points=stored_old->points();
    points.front().position.x=std::nextafter(points.front().position.x,std::numeric_limits<float>::infinity());
    points.front().appearances[0].quality=.6f;
    stored_old=mapping::scene::FeatureMap::create(0,std::move(points),mapping::scene::point_capacity);
  }
  memory.saveSubmap(old,LocalGrid(.1),stored_old);
  memory.saveSubmap(current,LocalGrid(.1),buildVisualScene(current));
  Eigen::Isometry3d committed_new=historical()*relative();
  if(kind==7)committed_new.translation().x()+=2.;
  memory.saveSubmapPoses({{1,historical().cast<float>()},{2,committed_new.cast<float>()}},
                         {{2,1,1,relative().inverse().cast<float>()}});
  memory.rebuildCommittedIndexes();
  const auto before=memory.committedRevision();
  check(bool(memory.loadCloud(0))&&bool(memory.loadScene(0)),"warm old payload caches");
  check(!memory.recallSpatial(old.bounds(),historical().cast<float>()).empty(),"warm spatial entries before retirement");
  force_rejection=kind==4?9:(kind==5?10:0);
  bool failed=false;std::optional<std::uint64_t> revision;
  try{revision=memory.refreshCoveredScene(1,2);}catch(...){failed=true;}
  force_rejection=0;
  if(kind==0||kind==6) {
    check(revision&&*revision==before+1&&!memory.loadCloud(0)&&!memory.loadScene(0)&&!memory.eligibleLoop(2,0,1),
          "committed refresh invalidates warm payload and candidate eligibility");
    const auto recalled=memory.recallSpatial(old.bounds(),historical().cast<float>());
    check(recalled.size()==1&&recalled.front().submap_id==1,"warm spatial lookup physically excludes retired target");
    memory.rebuildCommittedIndexes();
    check(memory.recallSpatial(old.bounds(),historical().cast<float>()).size()==1&&memory.submapPose(0).has_value(),
          "reconstruction preserves retired pose identity without restoring search membership");
  } else if(kind<=3||kind==7)check(!revision&&!failed&&memory.sceneActive(0)&&memory.committedRevision()==before,
                        "missing metric/novel appearance/wrong geometry/committed mismatch preserves old content");
  else check(failed&&memory.storageState()==MapDatabase::State::Failed,"refresh fault fences writer");
  memory.finish();
  MapDatabase reopened(path.string(),kind==4||kind==5?"recover":"resume",map_config_identity(config(true),navigation()));reopened.validateHistoricalRecords(.1);
  check(reopened.sceneActive(1)==(kind!=0&&kind!=5&&kind!=6),"atomic before/after refresh outcome on reopen");
  if(kind==0) {
    const auto updated=reopened.loadCloud(2);
    check((updated->front().mean.array()==current.lio().pcd->front().mean.array()).all()&&
          (relative()*updated->front().mean.cast<double>()-old.lio().pcd->front().mean.cast<double>()).norm()>.019,
          "fresh measured Gaussian update survives replacement/reopen, not just duplicate deletion");
  }
  reopened.finish();
}
void refreshContinuation(const fs::path &path) {
  seed(path,1,0,true);Fixture f;PoseGraphBackend b(config(true),navigation(),path.string(),{});
  auto root=f.frame(900,fresh(),false,true,false,0,true,false,true);
  check(b.attachFreshSessionAutomatically(root,LocalGrid(.1),77).status==AttachmentStatus::Attached,"B2 refresh root");
  b.beginContinuation(77);
  for(int i=0;i<8;++i) {
    auto q=f.frame(901+i,fresh(),false,true,false,0,true,false,true);LocalGrid grid(.1);
    check(b.submitContinuation(q,grid,77,true)==ContinuationAdmission::Accepted,"B2 refresh admission");
  }
  b.drain();check(b.continuationProgress().completed==8&&b.continuationProgress().ready_revision==b.graphRevision(),"B2 readiness includes refresh revisions");b.close();
  check(scalar(path,"SELECT count(*) FROM Node")==10&&scalar(path,"SELECT count(*) FROM LaserRecord")==4,"B2 temporal window bounds full-repeat active payload");
  MapDatabase db(path.string(),"resume",map_config_identity(config(true),navigation()));db.validateHistoricalRecords(.1);db.finish();
}
void refreshBackendFailure(const fs::path &path,int fault=9) {
  seed(path,1,0,true);Fixture f;PoseGraphBackend b(config(true),navigation(),path.string(),{});
  auto q=f.frame(900,fresh(),false,true,false,0,true,false,true);force_rejection=fault;
  bool reported=false;
  try{(void)b.attachFreshSessionAutomatically(q,LocalGrid(.1),77);}catch(const MapError &e){reported=e.outcome()==(fault==11?CommitOutcome::Unknown:CommitOutcome::Committed);}
  force_rejection=0;check(reported&&b.failed(),"refresh rollback after B1 must report already committed new observation and fence runtime");
  if(fault==11)check(!b.continuationProgress().committed_outcome_known&&b.continuationProgress().last_commit_outcome==CommitOutcome::Unknown,
                    "uncertain later revision must not be masked by known root commit");
  try{b.drain();}catch(...){} b.close();
  {MapDatabase recovery(path.string(),"recover",map_config_identity(config(true),navigation()));recovery.validateHistoricalRecords(.1);recovery.finish();}
  check(scalar(path,"SELECT count(*) FROM Node")==2&&scalar(path,"SELECT count(*) FROM LaserRecord")== (fault==11?1:2),"recovery resolves actual refresh commit outcome");
}
void refreshContinuationUnknown(const fs::path &path,bool resumed) {
  Fixture f;auto cfg=config(true);if(resumed)seed(path,1,0,true);else cfg.map_mode="new";
  PoseGraphBackend b(cfg,navigation(),path.string(),{});
  if(resumed) {
    auto root=f.frame(900,fresh(),false,true,false,0,true,false,true);
    check(b.attachFreshSessionAutomatically(root,LocalGrid(.1),77).status==AttachmentStatus::Attached,"uncertain B2 root");
    b.beginContinuation(77);
  }
  force_rejection=11;
  for(int i=0;i<(resumed?4:5);++i) {
    auto q=f.frame(resumed?901+i:i,fresh(),false,true,false,0,true,false,true);
    if(resumed) {LocalGrid grid(.1);check(b.submitContinuation(q,grid,77,true)==ContinuationAdmission::Accepted,"uncertain B2 admission");}
    else {b.addFrame(std::move(q));await([&]{return b.failed()||b.continuationProgress().completed==std::uint64_t(i+1);});}
  }
  bool failed=false;try{b.drain();}catch(...){failed=true;}force_rejection=0;
  check(failed&&b.failed()&&!b.continuationProgress().committed_outcome_known&&b.continuationProgress().last_commit_outcome==CommitOutcome::Unknown,
        "B2/new-map uncertainty survives prior known commits");
  const auto last_known=b.continuationProgress().committed_revision;
  b.close();
  MapDatabase recovered(path.string(),"recover",map_config_identity(config(true),navigation()));recovered.validateHistoricalRecords(.1);
  check(last_known&&recovered.committedRevision()==*last_known+1,"authoritative recovery finds the later committed scene revision");recovered.finish();
}
void corruptRefresh(const fs::path &source,const fs::path &root) {
  const std::vector<std::string> updates={"DELETE FROM SceneState WHERE node_id=2;","DELETE FROM LaserRecord WHERE node_id=2;",
    "INSERT INTO LaserRecord SELECT 1,gaussian_count,payload FROM LaserRecord WHERE node_id=2;",
    "UPDATE SceneState SET retired_revision=999999 WHERE node_id=1;"};
  for(std::size_t i=0;i<updates.size();++i) {
    auto path=root/("corrupt-refresh-"+std::to_string(i)+".db");fs::copy_file(source,path);
    sqlite3 *raw=nullptr;check(sqlite3_open_v2(path.c_str(),&raw,SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK,"tamper fixture");
    check(sqlite3_exec(raw,updates[i].c_str(),nullptr,nullptr,nullptr)==SQLITE_OK,"tamper scene membership fixture");check(sqlite3_close(raw)==SQLITE_OK,"close tamper");
    bool rejected=false;
    try{MapDatabase db(path.string(),"resume",map_config_identity(config(true),navigation()));db.validateHistoricalRecords(.1);db.finish();}catch(const MapError&){rejected=true;}
    check(rejected,"retirement metadata never excuses active missing/retired retained payload or invalid revision");
  }
}
void refreshNewMap(const fs::path &path) {
  Fixture f;auto cfg=config(true);cfg.map_mode="new";PoseGraphBackend b(cfg,navigation(),path.string(),{});
  for(int i=0;i<16;++i) {
    b.addFrame(f.frame(i,fresh(),false,true,false,0,true,false,true));
    await([&]{return b.failed()||b.continuationProgress().completed==std::uint64_t(i+1);});
    check(!b.failed(),"new-map revisit completion");
  }
  b.finish();
  check(scalar(path,"SELECT count(*) FROM Node")==16&&scalar(path,"SELECT count(*) FROM LaserRecord")==4,
        "new-map scene payload growth limited to temporal exclusion window on full revisits");
  MapDatabase db(path.string(),"resume",map_config_identity(config(true),navigation()));db.validateHistoricalRecords(.1);db.finish();
}
void child(const char *mode,const fs::path &path) {
  pid_t p=fork();check(p>=0,"fork");if(!p){execl("/proc/self/exe","sapphire_automatic_association_test",mode,path.c_str(),nullptr);_exit(120);}
  int status=0;check(waitpid(p,&status,0)==p&&status==0,"independent process stage");
}
}
namespace sapphire::database_detail {
void writableStorageTestPoint(const char *name,void *context) {
  std::string s(name);
  if(s=="refresh-after-delete"&&force_rejection==9)throw std::runtime_error("injected refresh rollback");
  if(s=="refresh-before-commit"&&force_rejection==11)uncertain_refresh_commit=true;
  if(s=="refresh-after-commit"&&force_rejection==10)throw std::runtime_error("injected refresh postcommit failure");
  if(s=="b1-after-bbs"||s=="b2-after-bbs")++bbs;
  if(s=="b2-before-preparation"&&force_rejection==8)throw std::runtime_error("injected retryable B2 preparation failure");
  if(s=="association-after-score") {
    {std::unique_lock<std::mutex> lock(barrier_mutex);if(block_score){score_entered=true;barrier_cv.notify_all();barrier_cv.wait(lock,[]{return !block_score;});}}
    ++scores;if(force_rejection==6)static_cast<gpu::PoseScore*>(context)->overlap=std::numeric_limits<double>::quiet_NaN();}
  if(s=="association-after-gicp") {++refinements;if(force_rejection==5)static_cast<GicpResult*>(context)->accepted=false;}
}
}
extern "C" int __real_sqlite3_exec(sqlite3*,const char*,int(*)(void*,int,char**,char**),void*,char**);
extern "C" int __wrap_sqlite3_exec(sqlite3 *db,const char *sql,int(*callback)(void*,int,char**,char**),void *arg,char **error) {
  const bool uncertain=sql&&std::strcmp(sql,"COMMIT;")==0&&uncertain_refresh_commit.exchange(false);
  const int rc=__real_sqlite3_exec(db,sql,callback,arg,error);
  return uncertain&&rc==SQLITE_OK?SQLITE_IOERR:rc;
}
int main(int argc,char **argv) try {
  if(argc==3) {
    const std::string stage(argv[1]);
    if(stage=="refresh-seed"){seed(argv[2],1,0,true);return 0;}
    if(stage=="refresh-cycle"){refreshCycle(argv[2]);return 0;}
    if(stage=="refresh-read"){refreshRead(argv[2]);return 0;}
    std::string mode(argv[1]);if(mode=="seed")seed(argv[2]);else if(mode=="attach")attach(argv[2],true);else if(mode=="read")reopen(argv[2]);else if(mode=="pipeline")pipelineAttach(argv[2]);else if(mode=="pipeline-read")pipelineRead(argv[2]);else throw std::runtime_error("mode");return 0;
  }
  char temp[]="/tmp/sapphire-auto-association-XXXXXX";check(mkdtemp(temp),"temp directory");fs::path root(temp);
  if(argc==2&&std::string(argv[1])=="--refresh") {
    for(int kind=0;kind<=7;++kind)directRefreshPair(root/("refresh-case-"+std::to_string(kind)+".db"),kind);
    corruptRefresh(root/"refresh-case-0.db",root);
    refreshBackendFailure(root/"refresh-backend-failure.db");
    refreshBackendFailure(root/"refresh-b1-unknown.db",11);
    refreshContinuationUnknown(root/"refresh-b2-unknown.db",true);
    refreshContinuationUnknown(root/"refresh-new-unknown.db",false);
    refreshContinuation(root/"refresh-continuation.db");
    refreshNewMap(root/"refresh-new-map.db");
    const auto path=root/"refresh-process.db";child("refresh-seed",path);
    std::uintmax_t warm_size=0;
    for(int i=0;i<12;++i){child("refresh-cycle",path);if(i==3)warm_size=fs::file_size(path);}
    child("refresh-read",path);
    const auto final_size=fs::file_size(path);
    check(final_size<=warm_size+64*1024,"retired SQLite pages reused on repeated visits (small graph metadata still grows)");
    std::cout<<"PASS scene refresh: 13 nodes, 1 active payload; warm_bytes="<<warm_size<<" final_bytes="<<final_size<<"; new-map temporal active bound=4; rollback/preservation/reopen\n";
    fs::remove_all(root);return 0;
  }
  if(argc==2&&std::string(argv[1])=="--frontend") {
    frontendCases(root);
    for(const char *mode:{"seed","pipeline","pipeline-read"})child(mode,root/"pipeline-process.db");
    fs::remove_all(root);std::cout<<"PASS automatic frontend: bounded pending views, ordered B1/B2, stop/cancel/errors and cross-process reopen\n";return 0;
  }
  for(int kind=1;kind<=5;++kind)rejection(root/("reject-"+std::to_string(kind)+".db"),kind);
  rejection(root/"rotation-conflict.db",7);
  seed(root/"agree.db",2);attach(root/"agree.db",false,2);
  seed(root/"numeric.db");{
    Fixture f;PoseGraphBackend b(config(),navigation(),(root/"numeric.db").string(),{});auto q=f.frame(900,fresh());
    const auto before=bytes(root/"numeric.db");force_rejection=6;bool thrown=false;
    try{(void)b.attachFreshSessionAutomatically(q,LocalGrid(.1),77);}catch(const std::runtime_error&){thrown=true;}
    check(thrown&&q.id()==900&&bytes(root/"numeric.db")==before,"invalid numerical score is error, not a negative place match");force_rejection=0;b.finish();
  }
  for(const char *mode:{"seed","attach","read"})child(mode,root/"process.db");
  fs::remove_all(root);std::cout<<"PASS automatic metric association, ambiguity/rejection/no-write, no BBS, independent-process B1/B2/reopen\n";
} catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
