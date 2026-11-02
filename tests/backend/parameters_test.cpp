#include <parameters.hpp>
#include <database/sqlite.hpp>
#include <database/memory.hpp>
#include <module/mapping.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <cmath>
#include <unistd.h>
using namespace sapphire;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
template<class F> void rejects(F action) {
  bool failed=false;
  try { action(); } catch (const std::invalid_argument&) { failed=true; }
  check(failed,"Invalid parameter was accepted");
}
template<class T> void invalid_values(T original, std::initializer_list<const char*> texts) {
  for (auto text:texts) {
    T value=original;
    rejects([&]{Parameters::parse({{"test",text}},"test",value);});
    check(value==original,"Failed parse overwrote a valid value");
  }
}
void startup_and_storage() {
  char pattern[] = "/tmp/parameter-startup-XXXXXX";
  const std::filesystem::path root = ::mkdtemp(pattern);
  struct Cleanup { std::filesystem::path path; ~Cleanup() { std::filesystem::remove_all(path); } } cleanup{root};
  const auto file = root / "params.toml";
  std::ofstream(file) << "[mapping.registration.lidar]\nmax_iterations = 4\n[mapping.pose_graph]\nmin_error_improvement = 1.2345678901234567\n";
  auto loaded = load_parameters(file, SensorParameters{}).pose_graph.mapping.parameters();
  check(loaded.size() == Parameters::getDefaultParameters().size(), "Loader did not fill defaults");
  check(loaded.at("registration.lidar.max_iterations") == "4" && loaded.at("node.generate_node_ids") == "true", "Startup layers lost values");
  check(std::stod(loaded.at("pose_graph.min_error_improvement")) == 1.2345678901234567, "Loader lost numeric precision");
  for (const auto* key : {"semantic.config_file", "mapping.working_directory", "relocalizer.bow.vocabulary_path", "relocalizer.bow.vocabulary_hash", "frame.min_feature_ratio", "bundle.pixel_variance_px2", "bundle.robust_kernel_delta"})
    rejects([&] { MappingParameters::from_record({{key, "1"}}); });
  rejects([&] { MappingParameters::from_record({{"node.navimap.cell_sze_m", "1"}}); });
  rejects([&] { MappingParameters::from_record({{"node.navimap.max_obstacle_height_m", "0.1"}}); });


  const auto path = (root / "map.db").string();
  const auto read = [&](const std::string& dbpath) {
    DBDriverSqlite3 db;
    check(db.openConnection(dbpath, false, true), "Cannot read parameter record");
    auto values = db.getLastParameters();
    db.closeConnection(false);
    return values;
  };
  const auto add_node = [](Memory& memory) {
    auto node = std::make_unique<Node>(0, -1, 0, 1, "", SE3f::identity());
    auto* admitted = memory.admit_node(std::move(node), sapphire::Mat66_t<double>::Identity());
    check(admitted != nullptr, "Node admission failed");
    return admitted->id();
  };
  int id = 0;
  {
    Memory memory(MappingParameters::from_record(loaded));
    check(memory.init(path), "Cannot initialize startup fixture");
    check(memory.getParameters().parameters().at("registration.lidar.max_iterations") == "4", "Constructor configuration was lost at database open");
    id = add_node(memory);
    memory.close(true);
  }
  auto expected = loaded;
  check(read(path) == expected, "Saved record differs from effective startup configuration");
  check(!expected.count("pose_graph.isam2.relinearize_threshold"), "Unused optimizer threshold is still emitted");
  // Emulate a map written before this parameter was retired. Opening and offline
  // maintenance preserve its provenance, while the running algorithm uses startup tuning.
  expected["pose_graph.isam2.relinearize_threshold"] = "0.125";
  {
    DBDriverSqlite3 db;
    check(db.openConnection(path), "Cannot write historical parameter fixture");
    db.addInfoAfterRun(0,0,0,0,expected);
  }
  {
    Mapping algorithm;
    algorithm.init({}, path);
    check(algorithm.getParameters().parameters().at("registration.lidar.max_iterations") == Parameters::getDefaultParameters().at("registration.lidar.max_iterations"),
          "Database history silently supplied startup tuning");
    check(!algorithm.getParameters().parameters().count("pose_graph.isam2.relinearize_threshold"), "Historical threshold entered active configuration");
    algorithm.close(false);
  }
  check(read(path) == expected, "Opening a historical map rewrote provenance");
  auto tool = loaded;
  tool["registration.lidar.max_iterations"] = "2";
  tool["memory.load_all_nodes_on_start"] = "true";
  Memory maintenance(MappingParameters::from_record(tool));
  check(maintenance.init(path), "Cannot initialize maintenance fixture");
  maintenance.setParameterRecord(expected);
  check(maintenance.getParameters().parameters().at("registration.lidar.max_iterations") == "2",
        "Preserving a record reconfigured the running algorithm");
  maintenance.deleteLocation(id);
  maintenance.close(true);
  check(read(path) == expected, "Maintenance settings leaked into original parameter record");
  {
    Mapping empty;
    empty.init(MappingParameters::from_record(tool), path);
    empty.close(false);
    check(read(path) == expected, "Opening an empty map overwrote its parameter record");
  }
  // A record override belongs to one database session, not the Memory object forever.
  const auto next = (root / "next.db").string();
  check(maintenance.init(next), "Cannot reopen maintenance object");
  add_node(maintenance);
  maintenance.close(true);
  check(read(next) == tool, "A previous database's record override survived reinitialization");
}


void sensor_config_tests() {
  char pattern[] = "/tmp/sensor-parameters-XXXXXX";
  const std::filesystem::path dir = ::mkdtemp(pattern);
  struct Cleanup { std::filesystem::path path; ~Cleanup() { std::filesystem::remove_all(path); } } cleanup{dir};
  const std::string common = "[imu]\ntopic='/imu'\nrate_hz=200\n[preprocess]\nmax_scan_duration=0.25\npoint_filter_num=2\nblind=0.3\n";
  const std::string first = "[lidar0]\ntype='airy'\ntopic='/front'\ntime_offset=0.012\nextrinsic_tran=[1,2,3]\nextrinsic_rota=[0,-1,0,1,0,0,0,0,1]\n";
  const std::string second = "[lidar1]\ntype='robosense'\ntopic='/rear'\ntime_offset=-0.002\nextrinsic_tran=[4,5,6]\nextrinsic_rota=[1,0,0,0,1,0,0,0,1]\n";
  auto load = [&](const std::string &text) {
    std::ofstream(dir / "sensors.toml") << text;
    return load_sensor_parameters(dir / "sensors.toml");
  };
  auto one = load(common + first);
  check(one.lidars.size() == 1 && one.lidars[0].lidar_type == 1, "Single sensor count/type");
  check(one.lidars[0].time_offset == .012 && std::abs(one.lidars[0].blind_squared - .09) < 1e-12, "Timing/filter config lost");
  auto two = load(common + first + second);
  check(two.lidars.size() == 2 && two.lidars[1].extrinsic_tran[0] == 4 && two.lidars[1].time_offset == -.002, "Second calibration lost");
  auto changed = [&](std::string from, std::string to) {
    auto text = common + first + second;
    const auto pos = text.find(from); check(pos != std::string::npos, "Bad fixture");
    text.replace(pos, from.size(), to); return text;
  };
  for (const auto &text : {
      common, common + second, changed("[lidar1]", "[lidar2]"), changed("type='robosense'", "type='livox'"),
      changed("topic='/rear'", "topic='/front'"), changed("topic='/imu'", "topic='/front'"),
      changed("extrinsic_tran=[4,5,6]", "extrinsic_tran=[]"), changed("extrinsic_rota=[1,0,0,0,1,0,0,0,1]", "extrinsic_rota=[1,0,0,0,1,0,0,0,-1]"),
      changed("time_offset=-0.002", "time_offset=nan"), changed("point_filter_num=2", "point_filter_num=0"),
      changed("blind=0.3", "blind=1e300"), changed("type='airy'", "type='typo'"), changed("rate_hz=200", "rate_hz=20"),
      common + first + second + "[sync]\nlidar_merge_hz=3\n", common + first + "[sync]\nqueue_size=1\n",
      common + first + "[unknown]\nkey=1\n"}) rejects([&] { load(text); });
  for (const auto *type : {"livox", "velodyne", "hesaixt", "hesai", "robosense", "airy"}) {
    auto text = common + first; const auto pos = text.find("type='airy'"); text.replace(pos, 11, std::string("type='") + type + "'");
    check(load(text).lidars.size() == 1, "Supported single sensor rejected");
  }
  for (int type = 0; type < 4; ++type) {
    auto text = common + first;
    text.replace(text.find("type='airy'"), 11, "type=" + std::to_string(type));
    check(load(text).lidars.front().lidar_type == type, "Numeric LiDAR selector lost");
  }
  for (const auto *type : {"-1", "4", "2147483648", "1.0", "true"}) {
    auto text = common + first;
    text.replace(text.find("type='airy'"), 11, std::string("type=") + type);
    rejects([&] { load(text); });
  }
  check(load(changed("type='airy'", "type=1")).lidars.size() == 2, "Numeric dual selector rejected");
  std::ofstream(dir / "algorithm.toml") << "[odometry]\nvoxel_size=0.7\n";
  auto params = load_parameters(dir / "algorithm.toml", one.lidars.front());
  check(params.sensor.extrinsic_tran[1] == 2 && params.odometry.voxel_size == .7, "Algorithm/sensor configuration crossed");
  std::ofstream(dir / "algorithm.toml") << "[sensor]\nlidar_type='livox'\n";
  rejects([&] { load_parameters(dir / "algorithm.toml", one.lidars.front()); });
}

int main() try {
  sensor_config_tests();
  const auto& defaults=Parameters::getDefaultParameters();
  check(MappingParameters::from_record({}).parameters() == defaults, "Default startup configuration changed");
  check(Parameters::deserialize(Parameters::serialize(defaults)) == defaults, "Parameter round trip");
  for (const auto& [key,value]:defaults)
    check(Parameters::getType(key).has_value() && !Parameters::getDescription(key).empty(), "Incomplete catalog");
  for (const auto* key : {"registration.feature.method", "preprocess.stereo_image_width", "node.navimap.cell_size_m",
                          "relocalizer.bow.vocabulary_path", "node.local_ba.max_iterations", "marker.enabled", "compression.detect.angle_deg", "compression.merge.radius_m", "registration.lidar.ba_enabled", "registration.lidar.fusion_enabled", "relocalizer.wm.enabled"}) {
    check(!defaults.count(key), "Retired parameter still in catalog");
    rejects([&]{MappingParameters::from_record({{key,"1"}});});
  }
  invalid_values(17,{""," ","garbage","7tail","2147483648","-2147483649","+-1","++1","1.5"});
  invalid_values(17u,{"-1","4294967296","7tail",""});
  invalid_values(1.5f,{"","1tail","1e100","nan","inf","-inf","1,25","+-1"});
  invalid_values(1.5,{"1e9999","1e-9999","nan","inf",""});
  invalid_values(false,{"","garbage","2","truefalse"});
  int n=17; check(!Parameters::parse({},"test",n) && n==17,"Missing value changed field");
  Parameters::parse({{"test","-2147483648"}},"test",n);
  check(n==std::numeric_limits<int>::min(),"Signed range narrowed");
  double x=0; Parameters::parse({{"test"," +1.25e-2 "}},"test",x); check(std::abs(x-.0125)<1e-12,"Scientific notation lost");
  const auto registration=RegistrationParams{}.withOverrides({{"registration.lidar.max_iterations","24"}});
  check(registration.lidar_max_iterations==24,"LiDAR configuration ignored");
  check(registration.withOverrides({{"registration.lidar.min_inliers","96"}}).lidar_max_iterations==24,"Partial update reset fields");
  for (const auto& entry: ParametersMap{{"registration.lidar.max_iterations","0"},{"registration.lidar.min_overlap","1.1"},
       {"registration.lidar.translation_sigma_m","0"},{"relocalizer.spatial.max_candidates","0"},
       {"pose_graph.isam2.relinearize_skip","0"},{"database.writer.commit_timeout_ms","0"}})
    rejects([&]{MappingParameters::from_record({entry});});
  const auto root=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  const auto sensors = sapphire::load_sensor_parameters(root / "core/config/sensors.toml");
  const auto params = sapphire::load_parameters(root / "core/config/params.toml", sensors.lidars.front());
  params.pose_graph.mapping.validate();
  for(const auto* record:{"registration.feature.method:0", "node.navimap.frame_grid_enabled:true", "external.setting:kept"}) {
    const auto historical=Parameters::deserialize(record);
    check(Parameters::serialize(historical)==record,"Provenance rewritten");
    rejects([&]{MappingParameters::from_record(historical);});
  }
  for(const auto* record:{"garbage", ":missing_name", "key:1;key:2"}) rejects([&]{Parameters::deserialize(record);});
  startup_and_storage();
  std::cout << "PASS LiDAR configuration, retired-key rejection and persisted provenance\n";
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
