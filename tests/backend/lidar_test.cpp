#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "database/sqlite.hpp"
#include "database/compression.hpp"
#include <cstring>
#include <limits>
#include "database/memory.hpp"
#include "solver/icp_solver.hpp"
#include "module/mapping.hpp"
#include "module/relocalizer.hpp"
#include "tools/logging.hpp"

namespace b = sapphire;
using sapphire::GaussianCloud;
using sapphire::GaussianCloudPtr;
void check(bool ok, const char* text) {
  if (!ok) throw std::runtime_error(text);
}
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
void bounded_matches() {
  auto target=cloud();
  auto query=std::make_shared<GaussianCloud>(*target);
  for(size_t i=0;i<query->size();++i) query->at(i).mean+=Eigen::Vector3f(float(i%5)-2,.13f,-.27f);
  const auto transform=truth();
  for(double radius:{.001,.5,2.0,100.0}) {
    const auto matches=b::lidar::match(target,query,transform,radius);
    size_t next=0;
    for(size_t i=0;i<query->size();++i) {
      const Eigen::Vector3d p=transform.rotationMatrix().cast<double>()*query->at(i).mean.cast<double>()+transform.translationVector().cast<double>();
      double nearest=std::numeric_limits<double>::infinity();size_t index=0;
      for(size_t j=0;j<target->size();++j) {
        const double distance=(p-target->at(j).mean.cast<double>()).squaredNorm();
        if(distance<nearest){nearest=distance;index=j;}
      }
      if(nearest<=radius*radius) {
        check(next<matches.size() && matches[next].query==i && matches[next].target==index,"bounded nearest agrees with brute force");++next;
      }
    }
    check(next==matches.size(),"bounded nearest excludes distant observations");
  }
  auto one=std::make_shared<GaussianCloud>(1,target->front());one->front().mean.setZero();
  auto boundary=std::make_shared<GaussianCloud>(*one);
  boundary->front().mean.x()=2;
  check(b::lidar::match(one,boundary,b::SE3f::identity(),2).size()==1,"inclusive correspondence radius");
  boundary->front().mean.x()=std::nextafter(2.0f,3.0f);
  check(b::lidar::match(one,boundary,b::SE3f::identity(),2).empty(),"outside correspondence radius");
  boundary->front().mean.setConstant(1e5f);
  check(b::lidar::match(one,boundary,b::SE3f::identity(),2).empty(),"remote candidate has no matches");
  auto empty=std::make_shared<GaussianCloud>();
  bool rejected=false;try{b::lidar::match(empty,boundary,b::SE3f::identity(),2);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"empty target remains invalid geometry");
}
void spatial() {
  const auto path = (std::filesystem::temp_directory_path() / std::filesystem::path("sapphire-spatial-" + std::to_string(getpid()) + ".db")).string();
  check(!std::filesystem::exists(path), "fresh spatial fixture");
  sapphire::ParametersMap config{{"relocalizer.spatial.radius_m", "2"},
                          {"relocalizer.spatial.max_candidates", "2"},
                          {"relocalizer.spatial.max_graph_hops", "0"}};
  {
    b::DBDriverSqlite3 database;
    check(database.openConnection(path), "create cold spatial fixture");
    for (int id : {7, 43, 100, 301}) {
      auto node = std::make_unique<b::Node>(id, id == 7 ? 7 : 0, id == 301 ? -1 : 0, id, "", pose(0));
      node->set_lidar_cloud(cloud());
      for (const auto [from, to] : {std::pair{7, 43}, std::pair{43, 100}, std::pair{43, 301}, std::pair{301, 9001}}) {
        const b::Link edge(from, to, b::Link::kOdometry, pose(0));
        if (id == from) node->addLink(edge);
        if (id == to) node->addLink(edge.inverse());
      }
      check(database.asyncSave(std::move(node)), "persist spatial node");
    }
    database.emptyTrashes(false);
    database.closeConnection(true);
  }
  {
    const auto typed = b::MappingParameters::from_record(config);
    b::Memory memory(typed);
    check(memory.init(path, false), "open spatial metadata");
    check(!memory.getNode(7), "fixture candidate remains cold");
    b::Relocalizer finder(memory, typed.relocalizer, typed.registration);
    b::Node query(9001, 0, 0, 9001, "", pose(0));
    query.set_lidar_cloud(cloud());
    auto reversed = sapphire::SE3f::fromRotationTranslation(Eigen::AngleAxisf(float(M_PI), Eigen::Vector3f::UnitZ()).toRotationMatrix(),
                                                            Eigen::Vector3f(.5f, 0, 0));
    std::map<int, sapphire::SE3f> poses{{7, reversed}, {43, pose(1)}, {100, pose(2.01f)}, {301, pose(.1f)}, {9001, pose(0)}};
    check(finder.candidates(query, poses) == std::vector<int>({7, 43}),
          "sparse IDs, cold metadata, reversed heading and retired node filtering");
    check(!memory.getNode(7), "candidate retrieval does not load Gaussian payload");
    auto shifted=poses;
    for(auto& [id,t]:shifted)t=reversed.compose(t);
    check(finder.candidates(query,shifted)==finder.candidates(query,poses),"candidate search invariant under common SE3 gauge change");
    query.addLink(b::Link(9001, 7, b::Link::kOdometry, pose(.5f), sapphire::Mat66_t<double>::Identity()));
    check(finder.candidates(query, poses) == std::vector<int>({43}), "direct neighbor exclusion");
    query.removeLink(7);
    poses.at(7) = pose(3);
    poses.at(100) = pose(.2f);
    check(finder.candidates(query, poses) == std::vector<int>({100, 43}), "optimized pose movement replaces spatial ranking");
    poses.at(7) = pose(.5f);
    poses.at(100) = pose(3);
    query.addLink(b::Link(9001, 301, b::Link::kOdometry, pose(0)));
    auto bounded = typed.relocalizer;
    bounded.spatial_max_graph_hops = 2;
    check(b::Relocalizer(memory, bounded, typed.registration).candidates(query, poses) == std::vector<int>({43}),
          "bounded metadata traversal crosses a cold intermediate node");
    bounded.spatial_max_graph_hops = 3;
    check(b::Relocalizer(memory, bounded, typed.registration).candidates(query, poses) == std::vector<int>({7, 43}),
          "graph hop bound includes exactly the reachable candidates");
    check(!memory.getNode(7) && !memory.getNode(43) && !memory.getNode(301),
          "graph traversal must not activate Gaussian payloads");
    double seconds = 0;
    memory.reactivateNodes({7, 43}, 2, seconds);
    memory.topograph().removeLink(7, 43);
    check(b::Relocalizer(memory, bounded, typed.registration).candidates(query, poses) == std::vector<int>({43}),
          "candidate traversal must observe committed edge removal without a constraint cache");
    memory.close(false);
  }
  std::filesystem::remove(path);
}

void archive_limits() {
  GaussianCloud large(30000, cloud()->front());
  for (size_t i = 0; i < large.size(); ++i) large[i].voxel_key.x = i;
  auto archive = b::lidar::encode(large);
  auto decoded = b::lidar::decode(archive);
  check(decoded->size() == large.size() && decoded->back().voxel_key.x == 29999,
        "archive preserves more than 24000 Gaussian units");
  b::Node node(1);
  node.set_lidar_cloud(decoded);
  check(node.lidar_cloud() == decoded, "Node has no archive resource policy");
  const auto path = (std::filesystem::temp_directory_path() / ("sapphire-large-node-" + std::to_string(getpid()) + ".db")).string();
  check(!std::filesystem::exists(path), "fresh large-node fixture");
  {
    b::DBDriverSqlite3 database;
    check(database.openConnection(path), "create large-node database");
    check(database.asyncSave(std::make_unique<b::Node>(node)), "save full large-node geometry");
    database.closeConnection(true);
  }
  {
    b::DBDriverSqlite3 database;
    check(database.openConnection(path), "reopen large-node database");
    std::unique_ptr<b::Node> restored(database.loadNode(1));
    check(restored && restored->lidar_cloud() && restored->lidar_cloud()->size() == large.size(),
          "database retains more than 24000 Gaussian units");
    database.closeConnection(false);
  }
  std::filesystem::remove(path);
  auto reject = [](const cv::Mat& damaged) {
    bool rejected = false;
    try { (void)b::lidar::decode(damaged); } catch (const std::exception&) { rejected = true; }
    check(rejected, "malformed archive rejected");
  };
  auto wrong_shape = archive.clone();
  const int huge_width = 16 + ((std::numeric_limits<int>::max() - 16) / 85) * 85;
  std::memcpy(wrong_shape.data + wrong_shape.total() - 8, &huge_width, sizeof(huge_width));
  reject(wrong_shape);  // Must not allocate the forged two-gigabyte trailer size.
  auto raw = b::uncompressData(archive);
  const std::uint64_t huge_count = (huge_width - 16) / 85;
  for (size_t i = 0; i < 8; ++i) raw.data[8 + i] = static_cast<unsigned char>(huge_count >> (8 * i));
  auto forged = b::compressData(raw);
  std::memcpy(forged.data + forged.total() - 8, &huge_width, sizeof(huge_width));
  reject(forged);  // Matching forged header and trailer still cannot drive allocation.
  std::memset(raw.data + 8, 0xff, 8);
  reject(b::compressData(raw));  // Count multiplication must not wrap.
  auto truncated = archive.clone();
  truncated.data[truncated.total() - 13] ^= 1;
  reject(truncated);  // checksum
  cv::Mat trailing(1, archive.cols + 1, CV_8UC1);
  std::memcpy(trailing.data, archive.data, archive.total() - 12);
  trailing.data[archive.total() - 12] = 0;
  std::memcpy(trailing.data + trailing.total() - 12, archive.data + archive.total() - 12, 12);
  reject(trailing);
}

void kernels() {
  archive_limits();
  spatial();
  auto target = cloud();
  auto decoded = b::lidar::decode(b::lidar::encode(*target));
  check(decoded->size() == target->size(), "geometry count");
  for (size_t i = 0; i < target->size(); ++i) {
    const auto& a = target->at(i);
    const auto& c = decoded->at(i);
    check(a.mean == c.mean && a.covariance == c.covariance && a.voxel_key == c.voxel_key && a.N == c.N && a.radius == c.radius &&
              a.is_plane == c.is_plane,
          "exact Gaussian archive");
  }
  auto bad = b::lidar::encode(*target);
  bad.data[0] ^= 0x3f;
  bool rejected = false;
  try {
    b::lidar::decode(bad);
  } catch (const std::exception&) {
    rejected = true;
  }
  check(rejected, "corrupt geometry rejected");
  const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(.19, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-.11, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(.07, Eigen::Vector3d::UnitX()))
                                       .toRotationMatrix();
  const Eigen::Vector3d translation(.45, -.27, .14);
  auto query = std::make_shared<GaussianCloud>();
  for (const auto& p : *target) query->push_back(p.transformed(rotation.transpose(), -rotation.transpose() * translation));
  const auto truth = sapphire::SE3f::fromRotationTranslation(rotation.cast<float>(), translation.cast<float>());
  b::RegistrationParams config;
  config.lidar_bbs_enabled = std::getenv("SAPPHIRE_TEST_BBS") != nullptr;
  const auto result = b::lidar::align(target, query, pose(.3, -.2, .1), config);
  check(!result.T_target_query.isNull(), result.failure.c_str());
  check((truth.matrix() - result.T_target_query.matrix()).norm() < .01, "asymmetric target-from-query direction");
  check(result.covariance && result.covariance->llt().info() == Eigen::Success, "finite SPD pose covariance");

}
sapphire::ParametersMap parameters() {
  return {{"registration.lidar.bbs_enabled", std::getenv("SAPPHIRE_TEST_BBS") ? "true" : "false"},
          {"relocalizer.path.max_nodes", "1"}, // This fixture repeats one synthetic cloud at unrelated poses.
          {"topograph.persistent_root_frames", "true"},
          {"memory.recent_nodes", "3"},
          {"memory.resident_budget_bytes", "65536"},
          {"relocalizer.spatial.radius_m", "2"},
          {"relocalizer.spatial.max_graph_hops", "0"},
          };
}
void write(const std::string& file, bool reject_loop = false) {
  check(!std::filesystem::exists(file), "test database must be fresh");
  b::Mapping mapping;
  auto settings = parameters();
  if (reject_loop) settings["pose_graph.max_constraint_error_sigma"] = "0.000001";
  mapping.init(b::MappingParameters::from_record(settings), file);
  auto geometry = cloud();
  for (int i = 0; i < 14; ++i) {
    if (i == 13) check(!mapping.getMemory()->getNode(1), "old loop target must be evicted before recall");
    const auto data = i == 13 ? transformedQuery() : geometry;
    check(mapping.process_node(std::make_unique<b::Node>(data, i + 1, pose(i == 13 ? .3f : float(i) * 4)), sapphire::Mat66_t<double>::Identity()), "pure LIO node admitted");
  }
  check(mapping.getLastLocationId() == 14, "all LIO nodes retained");
  bool has_loop = false;
  for (const auto& [id, link] : mapping.getMemory()->getLinks(14, true))
    if (link.type() == b::Link::kRelocalization && ((link.from() == 14 && link.to() == 1) || (link.from() == 1 && link.to() == 14))) {
      const auto relative = link.from() == 1 ? link.transform() : link.transform().inverse();
      check((relative.matrix() - truth().matrix()).norm() < .01, "asymmetric persisted factor direction");
      has_loop = true;
    }
  if (reject_loop) {
    check(!has_loop && mapping.getRelocalizationId() == 0, "Rejected loop leaked into committed links");
    check(mapping.getMemory()->getNode(1), "Rejection fixture must actually retrieve the cold candidate");
    for (const auto& [id, accepted] : mapping.getLocalOptimizedPoses()) {
      const auto expected = pose(id == 14 ? .3f : float(id - 1) * 4);
      check((accepted.matrix() - expected.matrix()).norm() < 1e-5f, "Rejected loop changed accepted map poses");
    }
    check((mapping.getMapCorrection().matrix() - sapphire::SE3f::identity().matrix()).norm() < 1e-5f,
          "Rejected loop changed odometry correction");
  } else {
    check(has_loop, "spatial pure LIO loop committed");
  }
  const auto node = mapping.getNodeCopy(1);
  check(node.has_lidar(), "node geometry retained");
  mapping.close();
}
void reopen(const std::string& file) {
  b::Mapping mapping;
  mapping.init(b::MappingParameters::from_record(parameters()), file);
  std::map<int, sapphire::SE3f> poses;
  std::multimap<int, b::Link> links;
  std::map<int, b::Node> nodes;
  mapping.getGraph(poses, links, true, true, &nodes);
  check(poses.size() >= 14, "historical graph restored");
  check(nodes.at(1).has_lidar() && nodes.at(1).lidar_cloud()->size() == 600, "historical LiDAR restored");
  size_t loops = 0;
  for (const auto& [id, link] : links)
    if (link.type() == b::Link::kRelocalization) {
      ++loops;
      const auto relative = link.from() == 1 ? link.transform() : link.transform().inverse();
      check((relative.matrix() - truth().matrix()).norm() < .01, "reopened asymmetric loop factor");
    }
  check(loops > 0, "historical loop persisted");
  const auto data = cloud();
  check(mapping.process_node(std::make_unique<b::Node>(data, 100, pose(3)), sapphire::Mat66_t<double>::Identity()), "continue after restart");
  check(mapping.getLastLocationId() > 14, "persistent identity monotonic");
  mapping.close();
}
int main(int argc, char** argv) {
  try {
    sapphire::tools::configure({});
    if (argc == 1) {
      bounded_matches();
      kernels();
      const auto file = (std::filesystem::temp_directory_path() / ("sapphire-rejected-loop-" + std::to_string(getpid()) + ".db")).string();
      write(file, true);
      b::DBDriverSqlite3 database;
      check(database.openConnection(file), "reopen rejected-loop graph");
      std::multimap<int, b::Link> links;
      database.loadLinks(14, links, std::nullopt);
      for (const auto& [id, link] : links) check(!link.isRelocalization(), "Rejected loop reached persistent storage");
      database.closeConnection(false);
      std::filesystem::remove(file);
    } else if (argc == 3 && std::string(argv[1]) == "write")
      write(argv[2]);
    else if (argc == 3 && std::string(argv[1]) == "reopen")
      reopen(argv[2]);
    else
      throw std::runtime_error("invalid arguments");
    std::cout << "PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
