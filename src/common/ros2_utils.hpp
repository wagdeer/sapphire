#pragma once

#include "common.hpp"
#include "lio_frame.hpp"
#include "pose_graph.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spdlog/spdlog.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using ImuMsg = sensor_msgs::msg::Imu;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using ImuConstPtr = ImuMsg::ConstSharedPtr;
using PointCloud2ConstPtr = PointCloud2Msg::ConstSharedPtr;
using PointCloudPublisher = rclcpp::Publisher<PointCloud2Msg>::SharedPtr;

inline rclcpp::Node::SharedPtr g_node;
inline PointCloudPublisher pub_scan;
inline PointCloudPublisher pub_cmap;
inline PointCloudPublisher pub_init;
inline PointCloudPublisher pub_curr_path;
inline rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
inline rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_odom;
inline rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_navi_map;
inline rclcpp::Subscription<ImuMsg>::SharedPtr sub_imu;
inline rclcpp::Subscription<PointCloud2Msg>::SharedPtr sub_pcl;

inline double stamp_to_sec(const builtin_interfaces::msg::Time &stamp)
{
  return rclcpp::Time(stamp).seconds();
}

inline builtin_interfaces::msg::Time stamp_from_sec(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const auto nsec = static_cast<int64_t>(std::llround(seconds * 1e9));
  stamp.sec = static_cast<int32_t>(nsec / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);
  return stamp;
}

inline rclcpp::Time now_time()
{
  if(g_node)
  {
    return g_node->get_clock()->now();
  }
  return rclcpp::Clock(RCL_SYSTEM_TIME).now();
}

inline double now_sec()
{
  return now_time().seconds();
}

template <typename T>
inline void pub_pl_func(T &pl, PointCloudPublisher &pub)
{
  if(!pub) return;
  pl.height = 1;
  pl.width = pl.size();
  PointCloud2Msg output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "odom_frame";
  output.header.stamp = now_time();
  pub->publish(output);
}

inline void pub_trajectory_func(const std::vector<TrajectoryPoint> &trajectory, PointCloudPublisher &pub)
{
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  cloud.reserve(trajectory.size());
  for(const TrajectoryPoint &point : trajectory)
  {
    pcl::PointXYZINormal output;
    output.x = point.x; output.y = point.y; output.z = point.z; output.curvature = point.distance; output.intensity = point.session;
    cloud.push_back(output);
  }
  pub_pl_func(cloud, pub);
}

inline void initialize_ros_publishers(const rclcpp::Node::SharedPtr &node)
{
  pub_cmap = node->create_publisher<PointCloud2Msg>("/map_cmap", rclcpp::QoS(100));
  pub_scan = node->create_publisher<PointCloud2Msg>("/map_scan", rclcpp::QoS(100));
  pub_init = node->create_publisher<PointCloud2Msg>("/map_init", rclcpp::QoS(100));
  pub_curr_path = node->create_publisher<PointCloud2Msg>("/map_path", rclcpp::QoS(100));
  pub_odom = node->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(100));
  pub_map_odom = node->create_publisher<nav_msgs::msg::Odometry>("/map_odom", rclcpp::QoS(10));
  pub_navi_map = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());
}

inline std::string current_time_filename()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time;
  localtime_r(&now_time, &local_time);

  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");
  return stream.str();
}

inline void prepare_output_directory(const rclcpp::Node::SharedPtr &node, const std::string &savepath, const std::string &filename)
{
  const std::filesystem::path output_path = std::filesystem::path(savepath) / filename;
  if(std::filesystem::exists(output_path))
  {
    spdlog::warn("Output directory already exists, clearing and overwriting: {}", output_path.string());
    std::filesystem::remove_all(output_path);
  }
  std::filesystem::create_directories(output_path);
}

class ResultOutput
{
public:
  static ResultOutput &instance()
  {
    static ResultOutput inst;
    return inst;
  }

  void pub_odom_func(StateGroup &xc)
  {
    Eigen::Quaterniond q_this(xc.R);
    Eigen::Vector3d t_this = xc.p;
    Eigen::Vector3d v_this = xc.R.transpose() * xc.v;
    const rclcpp::Time stamp = now_time();

    static std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    if(!br && g_node)
    {
      br = std::make_unique<tf2_ros::TransformBroadcaster>(g_node);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom_frame";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = t_this.x();
    odom.pose.pose.position.y = t_this.y();
    odom.pose.pose.position.z = t_this.z();
    odom.pose.pose.orientation.w = q_this.w();
    odom.pose.pose.orientation.x = q_this.x();
    odom.pose.pose.orientation.y = q_this.y();
    odom.pose.pose.orientation.z = q_this.z();
    odom.twist.twist.linear.x = v_this.x();
    odom.twist.twist.linear.y = v_this.y();
    odom.twist.twist.linear.z = v_this.z();
    if(pub_odom) 
    {
      pub_odom->publish(odom);
    }

    if(!br) return;

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = t_this.x();
    transform.transform.translation.y = t_this.y();
    transform.transform.translation.z = t_this.z();
    transform.transform.rotation.w = q_this.w();
    transform.transform.rotation.x = q_this.x();
    transform.transform.rotation.y = q_this.y();
    transform.transform.rotation.z = q_this.z();
    br->sendTransform(transform);
  }

  void pub_map_odom_func(const Eigen::Isometry3d &T_map_odom)
  {
    static std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    if(!br && g_node)
    {
      br = std::make_unique<tf2_ros::TransformBroadcaster>(g_node);
    }
    if(!br) return;

    const Eigen::Quaterniond rotation(T_map_odom.rotation());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now_time();
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom_frame";
    transform.transform.translation.x = T_map_odom.translation().x();
    transform.transform.translation.y = T_map_odom.translation().y();
    transform.transform.translation.z = T_map_odom.translation().z();
    transform.transform.rotation.w = rotation.w();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    br->sendTransform(transform);
  }

  void pub_map_pose_func(const Eigen::Isometry3d &T_map_base, double timestamp)
  {
    if(!pub_map_odom) return;
    const Eigen::Quaterniond rotation(T_map_base.rotation());
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp_from_sec(timestamp);
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = T_map_base.translation().x();
    odom.pose.pose.position.y = T_map_base.translation().y();
    odom.pose.pose.position.z = T_map_base.translation().z();
    odom.pose.pose.orientation.w = rotation.w();
    odom.pose.pose.orientation.x = rotation.x();
    odom.pose.pose.orientation.y = rotation.y();
    odom.pose.pose.orientation.z = rotation.z();
    pub_map_odom->publish(odom);
  }

  void pub_navi_map_func(const std::shared_ptr<const NavigationGrid> &map)
  {
    static size_t published_revision = 0;
    if(!pub_navi_map || !map || map->revision == published_revision) return;

    nav_msgs::msg::OccupancyGrid output;
    output.header.stamp = now_time();
    output.header.frame_id = "map";
    output.info.resolution = map->resolution;
    output.info.width = map->width;
    output.info.height = map->height;
    output.info.origin.position.x = map->origin_x;
    output.info.origin.position.y = map->origin_y;
    output.info.origin.orientation.w = 1.0;
    output.data = map->data;
    pub_navi_map->publish(output);
    published_revision = map->revision;
  }

  void pub_localtraj(vvec<double, 3> &pwld, double jour, StateGroup &x_curr, int cur_session, std::vector<TrajectoryPoint> &pcl_path)
  {
    pub_odom_func(x_curr);
    pcl::PointCloud<pcl::PointXYZINormal> pcl_send;
    pcl_send.reserve(pwld.size());
    for(Eigen::Vector3d &pw: pwld)
    {
      Eigen::Vector3d pvec = pw;
      pcl::PointXYZINormal ap;
      ap.x = pvec.x();
      ap.y = pvec.y();
      ap.z = pvec.z();
      pcl_send.push_back(ap);
    }
    pub_pl_func(pcl_send, pub_scan);

    Eigen::Vector3d pcurr = x_curr.p;

    TrajectoryPoint ap;
    ap.x = pcurr[0]; ap.y = pcurr[1]; ap.z = pcurr[2]; ap.distance = jour; ap.session = cur_session;
    pcl_path.push_back(ap);
    pub_trajectory_func(pcl_path, pub_curr_path);
  }

  void pub_localmap(int mgsize, int cur_session, std::vector<PointCloudPtr> &pvec_buf, std::vector<StateGroup> &x_buf, std::vector<TrajectoryPoint> &pcl_path, int win_base, int win_count)
  {
    pcl::PointCloud<pcl::PointXYZINormal> pcl_send;
    for(int i=0; i<mgsize; i++)
    {
      for(int j=0; j<pvec_buf[i]->size(); j+=3)
      {
        pointVar &pv = pvec_buf[i]->at(j);
        Eigen::Vector3d pvec = x_buf[i].R*pv.pnt + x_buf[i].p;
        pcl::PointXYZINormal ap;
        ap.x = pvec[0];
        ap.y = pvec[1];
        ap.z = pvec[2];
        ap.intensity = cur_session;
        pcl_send.push_back(ap);
      }
    }

    for(int i=0; i<win_count; i++)
    {
      Eigen::Vector3d pcurr = x_buf[i].p;
      pcl_path[i+win_base].x = pcurr[0];
      pcl_path[i+win_base].y = pcurr[1];
      pcl_path[i+win_base].z = pcurr[2];
    }

    pub_trajectory_func(pcl_path, pub_curr_path);
    pub_pl_func(pcl_send, pub_cmap);
  }
};

class FileReaderWriter
{
private:
  std::ofstream scan_pose_file_;
  std::ofstream keyframe_pose_file_;
  std::string keyframe_dir_;

public:
  static FileReaderWriter &instance()
  {
    static FileReaderWriter inst;
    return inst;
  }

  void open_session(const std::string &savepath, const std::string &session_name)
  {
    close_session();
    const std::filesystem::path session_dir = std::filesystem::path(savepath) / session_name;
    keyframe_dir_ = (session_dir / "keyframes").string();
    std::filesystem::create_directories(keyframe_dir_);
    scan_pose_file_.open((session_dir / "alidarState.txt").string());
    keyframe_pose_file_.open((std::filesystem::path(keyframe_dir_) / "poses.txt").string());
  }

  void save_pose(const MargiFrame &frame)
  {
    const StateGroup &state = frame.x;
    const Eigen::Quaterniond q(state.R);
    scan_pose_file_ << std::fixed << std::setprecision(6) << state.t << " "
      << std::setprecision(7) << state.p.x() << " " << state.p.y() << " "
      << state.p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
      << q.w() << " " << state.v.x() << " " << state.v.y() << " "
      << state.v.z() << " " << state.bg.x() << " " << state.bg.y() << " "
      << state.bg.z() << " " << state.ba.x() << " " << state.ba.y() << " "
      << state.ba.z() << " " << state.g.x() << " " << state.g.y() << " "
      << state.g.z();
    for(int i = 0; i < 6; i++) scan_pose_file_ << " " << frame.v6[i];
    scan_pose_file_ << std::endl;
  }

  void save_keyframe(const LioFrame &frame, int id)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(frame.pcd->size());
    for(const Eigen::Vector3f &point : *frame.pcd) cloud.emplace_back(point.x(), point.y(), point.z());
    pcl::io::savePCDFileBinary((std::filesystem::path(keyframe_dir_) / (std::to_string(id) + ".pcd")).string(), cloud);
    const Eigen::Quaterniond q(frame.T_odom_base.rotation());
    const Eigen::Vector3d &position = frame.T_odom_base.translation();
    keyframe_pose_file_ << std::fixed << std::setprecision(6)
      << id << " " << frame.timestamp << " " << std::setprecision(7)
      << position.x() << " " << position.y() << " " << position.z() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
      << std::endl;
  }

  void close_session()
  {
    if(scan_pose_file_.is_open()) scan_pose_file_.close();
    if(keyframe_pose_file_.is_open()) keyframe_pose_file_.close();
    keyframe_dir_.clear();
  }
};
