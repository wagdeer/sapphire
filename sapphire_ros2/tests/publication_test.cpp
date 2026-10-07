#define main b2_fixture_main
#include "../../sapphire/tests/continuation_test.cpp"
#undef main
#include "sapphire_ros2/sapphire_node.hpp"
#include "sapphire_ros2/checked_publish.hpp"
#include <rcl/error_handling.h>

namespace {
std::mutex transport_mutex;
std::function<bool(const std::string &)> transport_hook;
std::optional<sapphire_ros2::msg::RevisionedMapCorrection> revisioned_correction;
std::optional<nav_msgs::msg::Odometry> revisioned_pose;
std::optional<nav_msgs::msg::OccupancyGrid> revisioned_grid;
std::atomic_uint64_t geometry_matches{0};
std::optional<sapphire_ros2::msg::RevisionedMapPose> final_pose;
std::optional<sapphire_ros2::msg::RevisionedNavigationGrid> final_grid;
void transportHook(std::function<bool(const std::string &)> next) {
  std::lock_guard<std::mutex> l(transport_mutex);transport_hook.swap(next);
}
}
extern "C" rcl_ret_t __real_rcl_publish(const rcl_publisher_t*,const void*,rmw_publisher_allocation_t*);
extern "C" rcl_ret_t __wrap_rcl_publish(const rcl_publisher_t *pub,const void *message,rmw_publisher_allocation_t *allocation) {
  std::function<bool(const std::string &)> hook;
  {std::lock_guard<std::mutex> l(transport_mutex);hook=transport_hook;}
  const std::string topic=rcl_publisher_get_topic_name(pub);
  if (topic=="/sapphire/map_correction") revisioned_correction=*static_cast<const sapphire_ros2::msg::RevisionedMapCorrection*>(message);
  if (topic=="/map_odom/revisioned") { final_pose=*static_cast<const sapphire_ros2::msg::RevisionedMapPose*>(message);revisioned_pose=final_pose->pose; }
  if (topic=="/map/revisioned") { final_grid=*static_cast<const sapphire_ros2::msg::RevisionedNavigationGrid*>(message);revisioned_grid=final_grid->grid; }
  if (topic=="/map_odom") { check(revisioned_pose&&*revisioned_pose==*static_cast<const nav_msgs::msg::Odometry*>(message),"compatibility pose equals attributed source geometry and anchor time");++geometry_matches; }
  if (topic=="/map") { check(revisioned_grid&&*revisioned_grid==*static_cast<const nav_msgs::msg::OccupancyGrid*>(message),"compatibility grid equals attributed source geometry");++geometry_matches; }
  if(hook&&hook(topic))return RCL_RET_ERROR;
  return __real_rcl_publish(pub,message,allocation);
}
// TF lives in a shared library: --wrap=rcl_publish cannot intercept its internal
// calls. Wrap the actual broadcaster entry, then always delegate to real TF.
extern "C" void real_tf(tf2_ros::TransformBroadcaster*,const geometry_msgs::msg::TransformStamped&)
  asm("__real__ZN7tf2_ros20TransformBroadcaster13sendTransformERKN13geometry_msgs3msg17TransformStamped_ISaIvEEE");
extern "C" void wrap_tf(tf2_ros::TransformBroadcaster*,const geometry_msgs::msg::TransformStamped&)
  asm("__wrap__ZN7tf2_ros20TransformBroadcaster13sendTransformERKN13geometry_msgs3msg17TransformStamped_ISaIvEEE");
extern "C" void wrap_tf(tf2_ros::TransformBroadcaster *owner,const geometry_msgs::msg::TransformStamped &transform) {
  check(revisioned_correction&&transform==revisioned_correction->correction,"TF matches selected attributed correction");
  std::function<bool(const std::string &)> hook;
  {std::lock_guard<std::mutex> lock(transport_mutex);hook=transport_hook;}
  if(hook&&hook("/tf"))throw std::runtime_error("test TF local failure");
  real_tf(owner,transform);
}
namespace sapphire_ros {
struct SapphireNodeTestAccess {
  static void stopProducers(SapphireNode &n) {n.pipeline_->shutdown();}
  static void replace(SapphireNode &n) {sapphire::SlamPipelineTestAccess::replace(*n.pipeline_);}
  static void resetDomainStores(SapphireNode &n) {sapphire::SlamPipelineTestAccess::resetDomainStores(*n.pipeline_);}
  static auto failure(SapphireNode &n) {return n.pipeline_->producerFailure();}
  static bool paused(SapphireNode &n) {std::lock_guard<std::mutex> lock(n.output_mutex_);return n.output_resetting_;}

  static void pause(SapphireNode &n) {n.reset_output(true);}
  static void resume(SapphireNode &n) {n.reset_output(false);}
  static void awaitReset(SapphireNode &n) {
    std::unique_lock<std::mutex> lock(n.output_mutex_);
    check(n.output_cv_.wait_for(lock,std::chrono::seconds(30),[&]{return n.output_resetting_;}),"reset request entered output ownership");
  }
  static void map(SapphireNode &n,bool final=false) {n.attempt_map(final);}
  static auto pc(SapphireNode &n) {return n.correction_group_.published;}
  static auto pg(SapphireNode &n) {return n.navigation_group_.published;}
  static auto attempts(SapphireNode &n) {return n.correction_group_.online_attempts;}
  static void due(SapphireNode &n) {n.correction_group_.attempted={};n.navigation_group_.attempted={};n.next_grid_={};}
  static void add(SapphireNode &n,int id) {sapphire::SlamPipelineTestAccess::backend(*n.pipeline_).addFrame(frame(id,anchor(id)));}
  static void drain(SapphireNode &n) {n.pipeline_->drain();}
  static auto progress(SapphireNode &n) {return n.pipeline_->continuationProgress();}
  static auto incarnation(SapphireNode &n) {return n.incarnation_;}
  static void queue(SapphireNode &n,std::size_t slot,std::function<void()> fn) {n.enqueue_output(slot,std::move(fn));}
  static auto incrementalDrops(SapphireNode &n) {std::lock_guard<std::mutex> lock(n.output_mutex_);return n.incremental_dropped_;}
  static void realIncrement(SapphireNode &n) {
    auto points=std::make_shared<sapphire::vvec<double,3>>(100,Eigen::Vector3d::Ones());
    std::promise<void> done;auto result=done.get_future();
    n.enqueue_output(3,[&n,points,&done]{n.publish_local_map(points);done.set_value();},points->capacity()*sizeof(Eigen::Vector3d));
    check(result.wait_for(std::chrono::seconds(30))==std::future_status::ready,"real optional copy publication");result.get();
    std::unique_lock<std::mutex> lock(n.output_mutex_);n.output_cv_.wait(lock,[&]{return !n.output_active_;});
  }
  static PoseGraphBackend &resumed(SapphireNode &n,const fs::path &path) {
    pause(n);
    auto output=sapphire::SlamPipelineTestAccess::output(*n.pipeline_);
    n.pipeline_->drain();n.pipeline_->close();n.pipeline_.reset();
    // Keep the executor paused until the replacement unique owner is installed.
    output.owner_reset=[&n](bool begin){if(begin)n.reset_output(true);};
    SapphireParameters config;config.pose_graph=resumeConfig();config.pose_graph.database_path=path.string();config.general.save_map=0;
    n.pipeline_=std::make_unique<SlamPipeline>(config,std::move(output),std::make_pair(6,relation()));
    resume(n);return sapphire::SlamPipelineTestAccess::backend(*n.pipeline_);
  }
  static void finishParameter(SapphireNode &n) {n.set_parameter(rclcpp::Parameter("finish",true));n.finish_callback();}
};
}
using sapphire_ros::SapphireNode;
using Access=sapphire_ros::SapphireNodeTestAccess;
std::shared_ptr<SapphireNode> node(const fs::path &root,const std::string &name) {
  auto config=root/(name+".toml");
  std::ofstream f(config);f<<"[general]\nsave_path = \""<<root.string()<<"/\"\nsave_map = 0\n[map]\nmode = \"new\"\ndatabase_path = \""<<(root/(name+".db")).string()<<"\"\n[pose_graph]\nenabled = true\nupdate_period_sec = 0.02\n[navi_map]\nenabled = true\n";f.close();
  rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("algorithm_config",config.string())});
  return std::make_shared<SapphireNode>(options);
}
template<class Message> void saveWire(const fs::path &path,const Message &message) {
  rclcpp::Serialization<Message> serializer;rclcpp::SerializedMessage encoded;serializer.serialize_message(&message,&encoded);
  std::ofstream out(path,std::ios::binary);out.write(reinterpret_cast<const char*>(encoded.get_rcl_serialized_message().buffer),encoded.size());
}
void resumedProcess(const fs::path &root) {
  auto n=node(root,"resume-adapter");auto &backend=Access::resumed(*n,root/"resume.db");
  check(!backend.hasActiveCorrection(),"ROS resume starts with A2 only and needs fresh B1");
  Barrier selected;std::atomic_bool first{true};
  transportHook([&](const auto &topic){if(topic=="/sapphire/map_correction"&&first.exchange(false))selected.arrive();return false;});
  attach(backend,6);selected.wait();clearEvents();
  try {
    submit(backend,1);awaitEvent("b2-completed");submit(backend,2);awaitEvent("b2-completed",2);
    auto p=backend.continuationProgress();
    check(p.last_completed==42&&p.completed_revision==4&&p.root_source==40&&p.root_revision==2,"real B2 progresses while ROS sink blocked");
    const auto expected=semantic(backend,9);std::ofstream out(root/"resume.db.oracle",std::ios::binary);out.write(expected.data(),expected.size());
  } catch (...) {selected.release();throw;}
  auto finish=std::async(std::launch::async,[&]{return n->finish_checked();});selected.release();
  check(finish.get(),"explicit resume final local publication and checked close");transportHook({});
  const auto status=n->final_report();
  check(status.drain_target_available&&status.drain_revision==4&&status.drain_source==42&&status.generation==77&&status.root_available&&
        status.correction_revision==4&&status.navigation_revision==4&&status.close_success,"ROS final report exact resumed domain and F/S");
  check(final_pose&&final_grid&&final_pose->map_uuid==status.map_uuid&&final_pose->publisher_incarnation==status.publisher_incarnation&&
        final_pose->source_graph_revision==4&&final_grid->source_graph_revision==4,"final wire identity matches checked report");
  saveWire(root/"final-pose.cdr",*final_pose);saveWire(root/"final-grid.cdr",*final_grid);saveWire(root/"final-status.cdr",status);
  std::ofstream(root/"resume.db.uuid")<<status.map_uuid;
  std::ofstream(root/"final-witness.json")<<"{\"U\":\""<<status.map_uuid<<"\",\"E\":\""<<status.publisher_incarnation
    <<"\",\"g\":77,\"F\":4,\"S\":42,\"Pc\":4,\"Pg\":4,\"close_success\":true}\n";
  n.reset();
}
void rosChild(const char *mode,const fs::path &root) {
  auto pid=fork();check(pid>=0,"ROS oracle fork");
  if(!pid){execl("/proc/self/exe","sapphire_ros_publication_test",mode,root.c_str(),nullptr);_exit(120);}
  int status;check(waitpid(pid,&status,0)==pid&&status==0,std::string("ROS process ")+mode);
}
std::string exceptionText(std::exception_ptr error) {
  try {std::rethrow_exception(error);}catch(const std::exception &e){return e.what();}catch(...){return "nonstandard";}
}
void failedReplacement(const fs::path &root,const std::string &stage,bool close_failure) {
  auto n=node(root,std::string(close_failure?"close-":"construct-")+stage);Access::stopProducers(*n);
  const bool active=stage!="idle";Barrier selected;std::atomic_bool first{true},close_once{true};
  std::atomic_uint64_t required_calls{0},optional_calls{0};
  transportHook([&](const auto &topic) {
    if(topic=="/sapphire/map_correction"||topic=="/tf"||topic=="/map_odom/revisioned"||topic=="/map_odom"||topic=="/map/revisioned"||topic=="/map")++required_calls;
    if(active&&topic==(stage=="tf"?"/tf":"/sapphire/map_correction")&&first.exchange(false))selected.arrive();
    return false;
  });
  if(active) {clearEvents();Access::add(*n,0);selected.wait();}
  const auto old=Access::progress(*n);
  Access::resetDomainStores(*n);
  if(close_failure) setHook([&](const char *point,void*) {
    if(std::string(point)=="before-sqlite-close"&&close_once.exchange(false))throw std::runtime_error("R1 original old-owner close failure");
  });
  auto replacement=std::async(std::launch::async,[&] {
    try {Access::replace(*n);return std::exception_ptr{};}catch(...){return std::current_exception();}
  });
  if(active) {
    Access::awaitReset(*n);
    check(replacement.wait_for(std::chrono::seconds(0))!=std::future_status::ready,"replacement waits for selected old transport with CV mutex released");
    Access::queue(*n,3,[&]{++optional_calls;});
    check(Access::progress(*n).generation==old.generation,"reset crossing output/status keeps old owner identity");
    selected.release();
  }
  check(replacement.wait_for(std::chrono::seconds(30))==std::future_status::ready,"failed replacement reaches terminal pause state");
  const auto cause=replacement.get();setHook({});
  check(cause&&Access::failure(*n)==cause&&!Access::paused(*n),"R1 original failure retained and reset pause terminated");
  if(active) check(revisioned_correction&&final_pose&&revisioned_correction->generation==*old.generation&&final_pose->generation==*old.generation&&
                   revisioned_correction->map_uuid==old.map_uuid&&final_pose->map_uuid==old.map_uuid,"selected correction/TF finishes with old U/g");
  const auto settled_calls=required_calls.load();
  Access::queue(*n,3,[&]{++optional_calls;}); // Late producer callback after failed-reset acknowledgment.
  auto finish=std::async(std::launch::async,[&]{return n->finish_checked();});
  check(finish.wait_for(std::chrono::seconds(30))==std::future_status::ready,"R1 checked finish completes without test unpause");
  check(!finish.get()&&!n->finish_checked(),"failed and repeated checked finish return false");
  check(n->final_report().backend_error==exceptionText(cause),"checked report preserves original replacement cause");
  check(required_calls==settled_calls&&optional_calls==0,"no invalid old/new-domain output begins after failed reset acknowledgment");
  transportHook({});n.reset();
  std::cout<<"PASS R1/R3 failed replacement "<<(close_failure?"old-close":"construction")<<" selected="<<stage<<" repeated_finish=false original_cause="<<exceptionText(cause)<<'\n';
}
int main(int argc,char **argv) try {
  if(argc>1&&std::string(argv[1])=="--seed") {seed(fs::path(argv[2])/"resume.db",false);return 0;}
  if(argc>1&&std::string(argv[1])=="--read") {
    const auto result=b2_fixture_main(argc,argv);PoseGraphBackend b(resumeConfig(),{},argv[2],{});
    std::ifstream in(std::string(argv[2])+".uuid");std::string uuid;in>>uuid;auto p=b.continuationProgress();
    check(b.mapUuid()==uuid&&!p.generation&&!p.root_revision&&!p.completed_revision&&!b.hasActiveCorrection(),"independent restart preserves U, requires B1, restores no g or publication state");
    b.finish();return result;
  }
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  if(argc==3&&std::string(argv[1])=="--resume") {resumedProcess(argv[2]);rclcpp::shutdown();return 0;}
  if(argc==3&&std::string(argv[1])=="--incarnation") {
    auto child=node(argv[2],"incarnation-child");std::ofstream(fs::path(argv[2])/"incarnation.txt")<<Access::incarnation(*child);
    check(child->finish_checked(),"child close");child.reset();rclcpp::shutdown();return 0;
  }
  char temp[]="/tmp/sapphire-b3-ros-XXXXXX";check(mkdtemp(temp),"fixture");fs::path root=temp;
  rosChild("--seed",root);rosChild("--resume",root);oracle(root/"resume.db",root/"resume.db.oracle",9);
  std::cout<<"B3_ROS_ORACLE fixtures="<<root<<"\n";
  for(const auto &stage:{"idle","correction","tf"})failedReplacement(root,stage,false);
  for(const auto &stage:{"idle","correction"})failedReplacement(root,stage,true);
  auto n=node(root,"groups");Access::pause(*n);clearEvents();Access::add(*n,0);awaitEvent("b3-legacy-ready");Access::drain(*n);
  const auto f=Access::progress(*n).ready_revision;
  // First three correction actions succeed, standard Odometry deliberately fails.
  int calls=0;transportHook([&](const auto &topic){if(topic=="/map_odom"){++calls;return true;}return false;});
  Access::map(*n);check(!Access::pc(*n)&&Access::pg(*n)==f,"partial correction does not advance Pc; independent Pg succeeds");
  Access::map(*n);check(calls==1,"automatic retry cannot run before period");
  Access::due(*n);Access::map(*n);check(calls==2&&Access::attempts(*n)==2,"one automatic retry");
  Access::due(*n);Access::map(*n);check(calls==2,"exhausted online budget");
  n->retry_publication();n->retry_publication();Access::map(*n);check(calls==3&&Access::attempts(*n)==2,"explicit retry coalesces without budget refill");
  transportHook({});Access::resume(*n);check(n->finish_checked(),"final F succeeds after exhausted online retries without new input");
  check(Access::pc(*n)==f&&Access::pg(*n)==f&&rclcpp::ok(),"final output precedes owned context shutdown");
  check(geometry_matches>=2,"real adapter compatibility geometry comparison exercised");
  auto incarnation=Access::incarnation(*n);n.reset();
  auto partial=node(root,"partial-grid");Access::pause(*partial);clearEvents();Access::add(*partial,0);awaitEvent("b3-legacy-ready");Access::drain(*partial);
  transportHook([](const auto &topic){return topic=="/map";});Access::map(*partial);
  check(Access::pc(*partial)&&!Access::pg(*partial),"partial navigation leaves Pg unavailable despite revisioned grid success");
  transportHook({});Access::resume(*partial);check(partial->finish_checked(),"missing navigation final attempt");partial.reset();
  // Block the REAL required rcl_publish call while subsequent map heads become ready.
  auto lag=node(root,"required-lag");Barrier selected;std::atomic_bool first{true};
  transportHook([&](const auto &topic){if(topic=="/sapphire/map_correction"&&first.exchange(false))selected.arrive();return false;});
  clearEvents();Access::add(*lag,0);selected.wait();
  bool progressed=false;
  try {
    Access::add(*lag,1);awaitEvent("b3-legacy-ready",2);
    Access::add(*lag,2);awaitEvent("b3-legacy-ready",3);
    auto p=Access::progress(*lag);progressed=p.completed==3&&p.committed_revision==p.ready_revision;
  } catch (...) { selected.release();throw; }
  auto final=std::async(std::launch::async,[&]{return lag->finish_checked();});
  selected.release();check(final.get()&&progressed,"Y advances with real required sink blocked; final F drains without new input");
  check(lag->final_report().coalesced>=1&&!lag->final_report().pending&&!lag->final_report().in_flight,"required demand coalesces in finite state and settles");
  transportHook({});lag.reset();
  // Selected optional call and publisher lifetime: reset must wait for completion.
  auto lifetime=node(root,"lifetime");Barrier active;
  Access::queue(*lifetime,3,[&]{active.arrive();});active.wait();
  Access::queue(*lifetime,3,[]{});Access::queue(*lifetime,3,[]{});
  check(Access::incrementalDrops(*lifetime)==1,"distinct pending local-map increment replaced and counted");
  auto reset=std::async(std::launch::async,[&]{Access::pause(*lifetime);});
  Access::awaitReset(*lifetime);
  check(reset.wait_for(std::chrono::seconds(0))!=std::future_status::ready,"owner replacement waits for active output borrow");
  active.release();reset.get();
  check(Access::incrementalDrops(*lifetime)==2,"reset discard of distinct pending increment is counted");
  check(Access::incarnation(*lifetime)==incarnation,"process incarnation is shared by owners in this process");
  Access::resume(*lifetime);Access::realIncrement(*lifetime);check(lifetime->finish_checked(),"empty new map has no fabricated target");lifetime.reset();
  auto pid=fork();check(pid>=0,"process incarnation fork");
  if(!pid){execl("/proc/self/exe","sapphire_ros_publication_test","--incarnation",root.c_str(),nullptr);_exit(120);}
  int status;check(waitpid(pid,&status,0)==pid&&status==0,"fresh process incarnation child");
  std::ifstream text(root/"incarnation.txt");std::string next_incarnation;text>>next_incarnation;
  check(next_incarnation.size()==36&&next_incarnation!=incarnation,"process restart creates fresh volatile E");
  // Checked local publication with zero, late, and deliberately unspun subscribers.
  auto basic=std::make_shared<rclcpp::Node>("b3_adapter_test");
  auto pub=basic->create_publisher<nav_msgs::msg::OccupancyGrid>("/b3_adapter",rclcpp::QoS(1).transient_local());
  nav_msgs::msg::OccupancyGrid grid;grid.header.frame_id="map";grid.info.width=4000;grid.info.height=4000;grid.data.resize(16000000);
  sapphire_ros::checked_publish<nav_msgs::msg::OccupancyGrid>(pub,basic->get_node_base_interface()->get_context(),grid,16*1024*1024);
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer;rclcpp::SerializedMessage encoded;serializer.serialize_message(&grid,&encoded);
  check(encoded.size()<=16*1024*1024,"maximum grid encoded envelope");
  auto late=basic->create_subscription<nav_msgs::msg::OccupancyGrid>("/b3_adapter",rclcpp::QoS(1).transient_local(),[](nav_msgs::msg::OccupancyGrid::ConstSharedPtr){});
  // No executor spins this subscriber. Local completion remains the only contract.
  sapphire_ros::checked_publish<nav_msgs::msg::OccupancyGrid>(pub,basic->get_node_base_interface()->get_context(),grid,16*1024*1024);
  std::cout<<"B3 encoded_standard_grid_bytes="<<encoded.size()<<"\n";
  sapphire_ros2::msg::RevisionedNavigationGrid maximum;
  maximum.map_uuid=std::string(36,'a');maximum.publisher_incarnation=std::string(36,'b');maximum.source_graph_revision=UINT64_MAX;
  maximum.grid=grid;maximum.grid.header.frame_id=std::string(255,'f');
  rclcpp::Serialization<sapphire_ros2::msg::RevisionedNavigationGrid> revision_serializer;
  revision_serializer.serialize_message(&maximum,&encoded);
  check(encoded.size()<=16*1024*1024&&encoded.size()-16000000<=65536,"max attributed grid including bounded metadata fits envelope");
  std::cout<<"B3 encoded_revisioned_grid_bytes="<<encoded.size()<<"\n";
  auto lost=node(root,"lost");Access::pause(*lost);clearEvents();Access::add(*lost,0);awaitEvent("b3-legacy-ready");Access::drain(*lost);
  Access::finishParameter(*lost);check(lost->stop_requested()&&rclcpp::ok(),"finish parameter requests stop without context shutdown");
  rclcpp::shutdown();bool invalid=false;try{sapphire_ros::checked_publish<nav_msgs::msg::OccupancyGrid>(pub,basic->get_node_base_interface()->get_context(),grid);}catch(...){invalid=true;}
  check(invalid,"invalid context cannot report success");Access::resume(*lost);check(!lost->finish_checked(),"forced context loss fails final drain");lost.reset();
  std::cout<<"PASS B3 ROS groups, finite retries, final drain, lifetime, late/stalled/no subscribers, encoded cap, invalid context\n";
  return 0;
}catch(const std::exception&e){std::cerr<<"FAIL ROS B3 "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
