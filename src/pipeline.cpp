#include "pipeline.hpp"
#include "initialize.hpp"
#include "lm_optimizer.hpp"
#include "pose_graph.hpp"

using namespace std;

class VOXEL_SLAM
{
public:
  rclcpp::Node::SharedPtr node_;
  const VoxelSlamParameters parameters_;
  std::vector<TrajectoryPoint> pcl_path;
  StateGroup x_curr, extrin_para;
  ImuEstimator imu_estimator;
  ESKF eskf;
  VoxelMap voxel_map;
  double down_size_inv;

  int win_size;
  vector<StateGroup> x_buf;
  vector<PointCloudPtr> pvec_buf;
  vector<double> time_buf;
  vector<MeasGroup> init_buf;
  deque<ImuFactor*> imu_factor_buf;
  int win_count = 0, win_base = 0;

  mutex mtx_keyframe_queue;
  deque<MargiFrame, Eigen::aligned_allocator<MargiFrame>>
    buf_lba2keyframe, buf_reset_tail;
  int reset_flag = 0;
  int degrade_bound = 10;

  atomic_bool is_finish{false};
  atomic_int current_session_id{0};
  string filename, savepath;
  int is_save_map;
  unique_ptr<PoseGraphBackend> pose_graph;

  string backend_database_path() const
  {
    return (filesystem::path(savepath) / filename / "map.db").string();
  }

  VOXEL_SLAM(const rclcpp::Node::SharedPtr &node) : node_(node), parameters_(node), voxel_map(parameters_.odometry, parameters_.local_submap)
  {
    const GeneralParameters &general = parameters_.general;
    const SensorParameters &sensor = parameters_.sensor;
    const InitializerParameters &initializer = parameters_.initializer;
    const OdometryParameters &odometry = parameters_.odometry;
    const LocalSubmapParameters &local_submap_params = parameters_.local_submap;
    filename = current_time_filename();
    savepath = general.save_path;

    lidarproc_.configure(sensor);
    is_save_map = general.save_map;

    sub_imu = node_->create_subscription<ImuMsg>(general.imu_topic, rclcpp::SensorDataQoS().keep_last(80000),
      [](ImuConstPtr msg) { imu_handler(msg); });
    sub_pcl = node_->create_subscription<PointCloud2Msg>(general.lidar_topic, rclcpp::SensorDataQoS().keep_last(1000),
      [](PointCloud2ConstPtr msg) { pcl_handler(msg); });
    down_size_inv = odometry.down_size_inv;
    degrade_bound = odometry.degrade_bound;
    imu_estimator.configure(sensor, initializer, odometry);
    extrin_para.R = imu_estimator.Lid_rot_to_IMU;
    extrin_para.p = imu_estimator.Lid_offset_to_IMU;
    win_size = local_submap_params.win_size;

    if(is_save_map) {
      prepare_output_directory(node_, savepath, filename);
    }
    else if(parameters_.pose_graph.enabled) {
      filesystem::create_directories(filesystem::path(savepath) / filename);
    }
    pose_graph = make_unique<PoseGraphBackend>(
      parameters_.pose_graph,
      parameters_.navi_map,
      backend_database_path(),
      [](const shared_ptr<const NavigationGrid> &map) {
        ResultOutput::instance().pub_navi_map_func(map);
      });

    init_buf.reserve(win_size);
    spdlog::info("filename: {}", filename);
  }

  int initialization(MeasGroup &measures, Eigen::MatrixXd &hess, LidarFactor &voxhess, vvec<double, 3> &pwld)
  {
    deque<ImuMeas> &imus = measures.imu_buf;
    std::shared_ptr<std::vector<LidarPoint>> &pcl_curr = measures.lidar_cloud;
    auto orig = make_shared<std::vector<LidarPoint>>(*pcl_curr);
    if(imu_estimator.process(x_curr, *pcl_curr, imus) == 0) {
      return 0;
    }
    PointCloudPtr pptr(new PointCloud);
    const double init_down_size_inv = parameters_.initializer.down_size_inv;
    down_sampling_voxel(*pcl_curr, init_down_size_inv);
    var_init(extrin_para, *pcl_curr, pptr, parameters_.odometry.dept_err, parameters_.odometry.beam_err);
    eskf.observe_kdtree(x_curr, *pptr, init_down_size_inv);

    pwld.clear();
    pvec_update(pptr, x_curr, pwld);

    win_count++;
    x_buf.push_back(x_curr);
    pvec_buf.push_back(pptr);
    time_buf.push_back(measures.lidar_end_time);
    ResultOutput::instance().pub_localtraj(pwld, 0, x_curr, current_session_id.load(), pcl_path);

    if(win_count > 1)
    {
      imu_factor_buf.push_back(new ImuFactor(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
      imu_factor_buf[win_count-2]->push_imu(imus, imu_estimator.scale_gravity, parameters_.local_submap);
    }

    std::vector<LidarPoint> pl_mid = *orig;
    down_sampling_close(*orig, down_size_inv);
    if(orig->size() < 1000)
    {
      *orig = pl_mid;
      down_sampling_close(*orig, down_size_inv * 2.0);
    }

    sort(orig->begin(), orig->end(), [](const LidarPoint &x, const LidarPoint &y)
    {return x.time_offset < y.time_offset;});

    MeasGroup init_measures;
    init_measures.lidar_begin_time = measures.lidar_begin_time;
    init_measures.lidar_end_time = measures.lidar_end_time;
    init_measures.imu_buf = std::move(imus);
    init_measures.lidar_cloud = std::move(orig);
    init_buf.push_back(std::move(init_measures));

    int is_success = 0;
    if(win_count >= win_size)
    {
      is_success = Initialization::instance().motion_init(
        init_buf, &hess, voxhess, x_buf, voxel_map,
        pvec_buf, win_size, x_curr, imu_factor_buf, extrin_para,
        parameters_.initializer, parameters_.odometry, parameters_.local_submap,
        imu_estimator.scale_gravity);

      if(is_success == 0)
      {
        return -1;
      }
      init_buf.clear();
      return 1;
    }
    return 0;
  }

  void system_reset(const deque<ImuMeas> &imus)
  {
    voxel_map.reset();

    x_curr.setZero();
    x_curr.p = Eigen::Vector3d(0, 0, 30);
    imu_estimator.mean_acc.setZero();
    imu_estimator.init_num = 0;
    imu_estimator.init(imus);
    x_curr.g = -imu_estimator.mean_acc * imu_estimator.scale_gravity;

    for(int i=0; i<imu_factor_buf.size(); i++) {
      delete imu_factor_buf[i];
    }
    x_buf.clear();
    pvec_buf.clear();
    time_buf.clear();
    init_buf.clear();
    imu_factor_buf.clear();
    eskf.pl_tree.clear();

    win_base = 0; win_count = 0; pcl_path.clear();
    pub_trajectory_func(pcl_path, pub_cmap);
    spdlog::warn("Reset");
  }

  void thd_odometry()
  {
    vvec<double, 3> pwld;
    double down_sizes[3] = {0.1, 0.2, 0.4};
    Eigen::Vector3d last_pos(0, 0 ,0);
    double jour = 0;
    int counter = 0;

    int motion_init_flag = 1;
    eskf.pl_tree.clear();
    bool release_flag = false;
    int degrade_cnt = 0;
    LidarFactor voxhess(win_size);
    const int mgsize = 1;
    Eigen::MatrixXd hess;
    while(rclcpp::ok())
    {
      is_finish = declare_get_param<bool>(node_, "finish", false);
      if(is_finish)
      {
        break;
      }

      MeasGroup measures;
      if(!synchronizer.sync_packages(measures))
      {
        if(voxel_map.release_retired(1000) > 0)
        {
          malloc_trim(0);
        }
        else if(release_flag)
        {
          release_flag = false;
          voxel_map.prune(jour, 700.0);
        }
        else if(voxel_map.trim_window_pool(10000, 500) > 0)
        {
          malloc_trim(0);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      imu_estimator.pcl_beg_time = measures.lidar_begin_time;
      imu_estimator.pcl_end_time = measures.lidar_end_time;
      std::shared_ptr<std::vector<LidarPoint>> &pcl_curr = measures.lidar_cloud;
      deque<ImuMeas> &imus = measures.imu_buf;

      double t0 = now_sec();
      double t1=0, t2=0, t3=0, t4=0, t5=0, t6=0, t7=0, t8=0;

      if(motion_init_flag)
      {
        int init = initialization(measures, hess, voxhess, pwld);

        if(init == 1)
        {
          motion_init_flag = 0;
        }
        else
        {
          if(init == -1)
          {
            system_reset(init_buf.back().imu_buf);
          }
          continue;
        }
      }
      else
      {
        if(imu_estimator.process(x_curr, *pcl_curr, imus) == 0)
        {
          continue;
        }

        std::vector<LidarPoint> pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size_inv);

        if(pl_down.size() < 500)
        {
          pl_down = *pcl_curr;
          down_sampling_voxel(pl_down, down_size_inv * 2.0);
        }

        PointCloudPtr pptr(new PointCloud);
        var_init(extrin_para, pl_down, pptr, parameters_.odometry.dept_err, parameters_.odometry.beam_err);

        if(eskf.observe_voxelmap(x_curr, *pptr, voxel_map))
        {
          if(degrade_cnt > 0) degrade_cnt--;
        }
        else
        {
          degrade_cnt++;
        }

        pwld.clear();
        pvec_update(pptr, x_curr, pwld);
        ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, current_session_id.load(), pcl_path);

        t1 = now_sec();

        win_count++;
        x_buf.push_back(x_curr);
        pvec_buf.push_back(pptr);
        time_buf.push_back(measures.lidar_end_time);
        if(win_count > 1)
        {
          imu_factor_buf.push_back(new ImuFactor(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
          imu_factor_buf[win_count-2]->push_imu(imus, imu_estimator.scale_gravity, parameters_.local_submap);
        }
        
        voxhess.clear(); voxhess.win_size = win_size;

        voxel_map.cut_voxel_multi(pvec_buf[win_count-1], win_count-1, win_size, pwld);
        t2 = now_sec();

        voxel_map.recut_multi(win_count, x_buf, voxhess, parameters_.odometry.min_eigen_value, parameters_.local_submap.plane_eigen_value_thre);
        t3 = now_sec();

        if(degrade_cnt > degrade_bound)
        {
          degrade_cnt = 0;
          system_reset(imus);

          last_pos = x_curr.p; jour = 0;

          {
            lock_guard<mutex> lock(mtx_keyframe_queue);
            buf_reset_tail.swap(buf_lba2keyframe);
            reset_flag = 1;
          }

          motion_init_flag = 1;

          continue;
        }
      }

      if(win_count >= win_size)
      {
        t4 = now_sec();
        
        LI_BA_Optimizer opt_lsv(parameters_.local_submap);
        opt_lsv.damping_iter(x_buf, voxhess, imu_factor_buf, &hess);

        Eigen::Matrix<double, 6, 1> margi_variance = hess.block<POSE_DOF, POSE_DOF>(0, STATE_DOF).diagonal();
        for(int i=0; i<6; i++)
        {
          margi_variance[i] = 1.0 / fabs(margi_variance[i]);
        }

        x_curr.R = x_buf[win_count-1].R;
        x_curr.p = x_buf[win_count-1].p;
        t5 = now_sec();

        ResultOutput::instance().pub_localmap(mgsize, current_session_id.load(), pvec_buf, x_buf, pcl_path, win_base, win_count);

        voxel_map.marginalize(jour, win_count, mgsize, x_buf, voxhess);
        t6 = now_sec();

        {
          lock_guard<mutex> lock(mtx_keyframe_queue);
          buf_lba2keyframe.emplace_back(x_buf[0], std::move(pvec_buf[0]), time_buf[0], margi_variance);
        }

        if((win_base + win_count) % 10 == 0)
        {
          double spat = (x_curr.p - last_pos).norm();
          if(spat > 0.5)
          {
            jour += spat;
            last_pos = x_curr.p;
            release_flag = true;
          }
        }

        for(int i=mgsize; i<win_count; i++)
        {
          x_buf[i-mgsize] = x_buf[i];
          time_buf[i-mgsize] = time_buf[i];
          PointCloudPtr pvec_tem = pvec_buf[i-mgsize];
          pvec_buf[i-mgsize] = pvec_buf[i];
          pvec_buf[i] = pvec_tem;
        }

        for(int i=win_count-mgsize; i<win_count; i++)
        {
          x_buf.pop_back();
          pvec_buf.pop_back();
          time_buf.pop_back();

          delete imu_factor_buf.front();
          imu_factor_buf.pop_front();
        }

        win_base += mgsize; win_count -= mgsize;
      }
    }

    voxel_map.clear();
    while(voxel_map.release_retired(1000) > 0) {}
    malloc_trim(0);
  }

  void thd_mapping()
  {
    KeyframeBuffer keyframe_buffer(parameters_.pose_graph.keyframe_voxel_size_inv);
    int buf_base = 0;
    const string initial_filename = filename;
    if(is_save_map)
    {
      FileReaderWriter::instance().open_session(savepath, filename);
    }
    while(true)
    {
      deque<MargiFrame, Eigen::aligned_allocator<MargiFrame>> reset_tail;
      optional<MargiFrame> margi_frame;
      bool switch_session = false;
      {
        lock_guard<mutex> lock(mtx_keyframe_queue);
        if(reset_flag == 1)
        {
          reset_flag = 0;
          reset_tail.swap(buf_reset_tail);
          switch_session = true;
        }
        else if(!buf_lba2keyframe.empty())
        {
          margi_frame.emplace(std::move(buf_lba2keyframe.front()));
          buf_lba2keyframe.pop_front();
        }
      }

      if(switch_session)
      {
        if(is_save_map)
        {
          for(const MargiFrame &frame : reset_tail)
          {
            FileReaderWriter::instance().save_pose(frame);
          }
        }
        reset_tail.clear();
        keyframe_buffer.clear();

        const int session_id = current_session_id.fetch_add(1) + 1;
        filename = initial_filename + to_string(session_id);
        if(is_save_map)
        {
          prepare_output_directory(node_, savepath, filename);
          FileReaderWriter::instance().open_session(savepath, filename);
        }
        else if(parameters_.pose_graph.enabled)
        {
          filesystem::create_directories(filesystem::path(savepath) / filename);
        }

        buf_base = 0;
        pose_graph = make_unique<PoseGraphBackend>(parameters_.pose_graph, parameters_.navi_map, backend_database_path(),
          [](const shared_ptr<const NavigationGrid> &map)
          {
            ResultOutput::instance().pub_navi_map_func(map);
          });
        continue;
      }

      if(!margi_frame)
      {
        if(is_finish.load())
        {
          lock_guard<mutex> lock(mtx_keyframe_queue);
          if(buf_lba2keyframe.empty() && buf_reset_tail.empty() && reset_flag == 0)
          {
            break;
          }
        }
        if(!rclcpp::ok()) break;
        usleep(10000);
        continue;
      }

      if(is_save_map)
      {
        FileReaderWriter::instance().save_pose(*margi_frame);
      }
      buf_base++;

      LioFrame keyframe;
      if(!keyframe_buffer.push(std::move(*margi_frame), keyframe)) continue;

      pose_graph->addFrame(keyframe.pcd, keyframe.T_odom_base, keyframe.timestamp);
      const Eigen::Isometry3d T_map_odom = pose_graph->T_map_odom();
      ResultOutput::instance().pub_map_odom_func(T_map_odom);
      ResultOutput::instance().pub_map_pose_func(T_map_odom * keyframe.T_odom_base, keyframe.timestamp);
      if(is_save_map)
      {
        FileReaderWriter::instance().save_keyframe(keyframe, buf_base - 1);
      }
    }

    if(is_save_map) FileReaderWriter::instance().close_session();
    malloc_trim(0);
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("cmn_sapphire");

  initialize_ros_publishers(g_node);
  
  VOXEL_SLAM vs(g_node);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(g_node);
  thread thread_executor([&executor]() { executor.spin(); });
  thread thread_mapping(&VOXEL_SLAM::thd_mapping, &vs);
  vs.thd_odometry();
  thread_mapping.join();

  rclcpp::shutdown();
  executor.cancel();
  thread_executor.join();
  return 0;
}

