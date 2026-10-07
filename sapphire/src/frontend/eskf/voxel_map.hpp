#pragma once

#include <spdlog/spdlog.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/common.hpp"
#include "frontend/eskf/lidar_factor.hpp"
#include "parameters.h"

namespace sapphire {

using std::lock_guard;
using std::make_unique;
using std::mutex;
using std::pair;
using std::ref;
using std::thread;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

inline void Bf_var(const pointVar &pv, Eigen::Matrix<double, 9, 9> &bcov, const Eigen::Vector3d &vec) {
  Eigen::Matrix<double, 6, 3> Bi;
  Bi << 2 * vec(0), 0, 0, vec(1), vec(0), 0, vec(2), 0, vec(0), 0, 2 * vec(1), 0, 0, vec(2), vec(1), 0, 0, 2 * vec(2);
  Eigen::Matrix<double, 6, 3> Biup = Bi * pv.var;
  bcov.block<6, 6>(0, 0) = Biup * Bi.transpose();
  bcov.block<6, 3>(0, 6) = Biup;
  bcov.block<3, 6>(6, 0) = Biup.transpose();
  bcov.block<3, 3>(6, 6) = pv.var;
}

class SlideWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vector<PointCloud> points;
  vector<PointCluster> pcrs_local;

  SlideWindow(int wdsize) {
    pcrs_local.resize(wdsize);
    points.resize(wdsize);
    // Most temporal slots are empty; deepest leaves only retain moments.
    // Allocate point storage when a slot actually receives points, then reuse it.
  }

  std::size_t allocatedBytes() const {
    std::size_t bytes = sizeof(*this) + points.capacity() * sizeof(PointCloud) + pcrs_local.capacity() * sizeof(PointCluster);
    for (const auto &slot : points) bytes += slot.capacity() * sizeof(pointVar);
    return bytes;
  }

  void resize(int wdsize) {
    if (points.size() != wdsize) {
      points.resize(wdsize);
      pcrs_local.resize(wdsize);
    }
  }

  void clear() {
    int wdsize = points.size();
    for (int i = 0; i < wdsize; i++) {
      points[i].clear();
      pcrs_local[i].clear();
    }
  }
};

class OctoTree {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline static int max_layer = 2;
  inline static double voxel_size = 1.0;
  inline static double voxel_size_inv = 1.0;
  inline static vector<int> window_order;
  static constexpr int min_points = 5;
  static constexpr int max_points = 100;

  static void configure(const OdometryParameters &odometry, const LocalSubmapParameters &local_submap) {
    max_layer = local_submap.max_layer;
    voxel_size = odometry.voxel_size;
    voxel_size_inv = odometry.voxel_size_inv;
    window_order.resize(local_submap.win_size);
    for (int i = 0; i < local_submap.win_size; ++i) {
      window_order[i] = i;
    }
  }

  static void reset_window_order() {
    for (size_t i = 0; i < window_order.size(); ++i) {
      window_order[i] = static_cast<int>(i);
    }
  }

  static void advance_window_order(int count) {
    const int size = static_cast<int>(window_order.size());
    for (int &index : window_order) {
      index += count;
      if (index >= size) {
        index -= size;
      }
    }
  }

  SlideWindow *sw = nullptr;
  PointCluster pcr_add;
  // Symmetric moment covariance: upper triangle, column by column.
  // Keep double precision; expand only when propagating plane uncertainty.
  Eigen::Matrix<double, 45, 1> cov_add;

  PointCluster pcr_fix;
  PointCloud point_fix;

  int layer, octo_state, wdsize;
  VOXEL_LOC voxel_key;
  unique_ptr<OctoTree> leaves[8];
  double voxel_center[3];
  double jour = 0;
  float quater_length;

  Plane plane;
  bool isexist = false;

  Eigen::Vector3d eig_value;
  Eigen::Matrix3d eig_vector;

  int last_num = 0, opt_state = -1;
  mutex mVox;

  OctoTree(int _l, int _w, const VOXEL_LOC &_key = VOXEL_LOC()) : layer(_l), octo_state(0), wdsize(_w), voxel_key(_key) {
    cov_add.setZero();
  }

  ~OctoTree() { delete sw; }

  OctoTree(const OctoTree &) = delete;
  OctoTree &operator=(const OctoTree &) = delete;

  OctoTree *child(int leafnum) {
    if (leaves[leafnum] == nullptr) {
      const int x = (leafnum >> 2) & 1;
      const int y = (leafnum >> 1) & 1;
      const int z = leafnum & 1;
      const auto child_coordinate = [](int64_t parent, int bit) {
        if (parent > std::numeric_limits<int64_t>::max() / 2 || parent < std::numeric_limits<int64_t>::min() / 2) {
          throw std::overflow_error("Octree child voxel coordinate overflow");
        }
        return 2 * parent + bit;
      };
      if (voxel_key.level == std::numeric_limits<int>::max()) {
        throw std::overflow_error("Octree child voxel level overflow");
      }
      const VOXEL_LOC child_key(child_coordinate(voxel_key.x, x), child_coordinate(voxel_key.y, y), child_coordinate(voxel_key.z, z),
                                voxel_key.level + 1);
      leaves[leafnum] = make_unique<OctoTree>(layer + 1, wdsize, child_key);
      leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * x - 1) * quater_length;
      leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * y - 1) * quater_length;
      leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * z - 1) * quater_length;
      leaves[leafnum]->quater_length = quater_length / 2;
    }
    return leaves[leafnum].get();
  }

  inline void push(int ord, const pointVar &pv, const Eigen::Vector3d &pw, vector<SlideWindow *> &sws) {
    lock_guard<mutex> lock(mVox);
    if (sw == nullptr) {
      if (sws.size() != 0) {
        sw = sws.back();
        sws.pop_back();
        sw->resize(wdsize);
      } else {
        sw = new SlideWindow(wdsize);
      }
    }
    if (!isexist) {
      isexist = true;
    }

    int mord = window_order[ord];
    if (layer < max_layer) {
      sw->points[mord].push_back(pv);
    }
    sw->pcrs_local[mord].push(pv.pnt);
    pcr_add.push(pw);
    Eigen::Matrix<double, 9, 9> Bi;
    Bf_var(pv, Bi, pw);
    add_moment_covariance(Bi);
  }

  inline void push_fix(pointVar &pv) {
    if (layer < max_layer) {
      point_fix.push_back(pv);
    }
    pcr_fix.push(pv.pnt);
    pcr_add.push(pv.pnt);
    Eigen::Matrix<double, 9, 9> Bi;
    Bf_var(pv, Bi, pv.pnt);
    add_moment_covariance(Bi);
  }

  inline void push_fix_novar(pointVar &pv) {
    if (layer < max_layer) {
      point_fix.push_back(pv);
    }
    pcr_fix.push(pv.pnt);
    pcr_add.push(pv.pnt);
  }

  inline bool plane_judge(Eigen::Vector3d &eig_values, double eigen_threshold, const vector<double> &plane_thresholds) {
    return (eig_values[0] < eigen_threshold && (eig_values[0] / eig_values[2]) < plane_thresholds[layer]);
  }

  void allocate(int ord, const pointVar &pv, const Eigen::Vector3d &pw, vector<SlideWindow *> &sws) {
    if (octo_state == 0) {
      push(ord, pv, pw, sws);
    } else {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++) {
        if (pw[k] > voxel_center[k]) {
          xyz[k] = 1;
        }
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      child(leafnum)->allocate(ord, pv, pw, sws);
    }
  }

  void allocate_fix(pointVar &pv) {
    if (octo_state == 0) {
      push_fix_novar(pv);
    } else if (layer < max_layer) {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++) {
        if (pv.pnt[k] > voxel_center[k]) {
          xyz[k] = 1;
        }
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      child(leafnum)->allocate_fix(pv);
    }
  }

  void fix_divide(vector<SlideWindow *> &sws) {
    for (pointVar &pv : point_fix) {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++) {
        if (pv.pnt[k] > voxel_center[k]) {
          xyz[k] = 1;
        }
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      child(leafnum)->push_fix(pv);
    }
  }

  void subdivide(int si, StateGroup &xx, vector<SlideWindow *> &sws) {
    for (pointVar &pv : sw->points[window_order[si]]) {
      Eigen::Vector3d pw = xx.R * pv.pnt + xx.p;
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++) {
        if (pw[k] > voxel_center[k]) {
          xyz[k] = 1;
        }
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      child(leafnum)->push(si, pv, pw, sws);
    }
  }

  void add_moment_covariance(const Eigen::Matrix<double, 9, 9> &covariance) {
    int index = 0;
    for (int col = 0; col < 9; ++col)
      for (int row = 0; row <= col; ++row) cov_add[index++] += covariance(row, col);
  }

  Eigen::Matrix<double, 9, 9> moment_covariance() const {
    Eigen::Matrix<double, 9, 9> covariance;
    int index = 0;
    for (int col = 0; col < 9; ++col)
      for (int row = 0; row <= col; ++row) covariance(row, col) = covariance(col, row) = cov_add[index++];
    return covariance;
  }

  void plane_update() {
    const auto covariance = moment_covariance();
    plane.center = pcr_add.v / pcr_add.N;
    int l = 0;
    Eigen::Vector3d u[3] = {eig_vector.col(0), eig_vector.col(1), eig_vector.col(2)};
    double nv = 1.0 / pcr_add.N;

    Eigen::Matrix<double, 3, 9> u_c;
    u_c.setZero();
    for (int k = 0; k < 3; k++) {
      if (k != l) {
        Eigen::Matrix3d ukl = u[k] * u[l].transpose();
        Eigen::Matrix<double, 1, 9> fkl;
        fkl.head(6) << ukl(0, 0), ukl(1, 0) + ukl(0, 1), ukl(2, 0) + ukl(0, 2), ukl(1, 1), ukl(1, 2) + ukl(2, 1), ukl(2, 2);
        fkl.tail(3) = -(u[k].dot(plane.center) * u[l] + u[l].dot(plane.center) * u[k]);

        u_c += nv / (eig_value[l] - eig_value[k]) * u[k] * fkl;
      }
    }

    Eigen::Matrix<double, 3, 9> Jc = u_c * covariance;
    plane.plane_var.block<3, 3>(0, 0) = Jc * u_c.transpose();
    Eigen::Matrix3d Jc_N = nv * Jc.block<3, 3>(0, 6);
    plane.plane_var.block<3, 3>(0, 3) = Jc_N;
    plane.plane_var.block<3, 3>(3, 0) = Jc_N.transpose();
    plane.plane_var.block<3, 3>(3, 3) = nv * nv * covariance.block<3, 3>(6, 6);
    plane.normal = u[0];
    plane.radius = eig_value[2];
  }

  void recut(int win_count, vector<StateGroup> &x_buf, vector<SlideWindow *> &sws, double eigen_threshold, const vector<double> &plane_thresholds) {
    if (octo_state == 0) {
      if (layer >= 0) {
        opt_state = -1;
        if (pcr_add.N <= min_points) {
          plane.is_plane = false;
          return;
        }
        if (!isexist || sw == nullptr) {
          return;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
        eig_value = saes.eigenvalues();
        eig_vector = saes.eigenvectors();
        plane.is_plane = plane_judge(eig_value, eigen_threshold, plane_thresholds);

        if (plane.is_plane) {
          return;
        } else if (layer >= max_layer) {
          return;
        }
      }

      if (pcr_fix.N != 0) {
        fix_divide(sws);
        PointCloud().swap(point_fix);
      }

      for (int i = 0; i < win_count; i++) {
        subdivide(i, x_buf[i], sws);
      }

      sw->clear();
      sws.push_back(sw);
      sw = nullptr;
      octo_state = 1;
    }

    for (int i = 0; i < 8; i++) {
      if (leaves[i] != nullptr) {
        leaves[i]->recut(win_count, x_buf, sws, eigen_threshold, plane_thresholds);
      }
    }
  }

  void margi(int win_count, int mgsize, vector<StateGroup> &x_buf, const LidarFactor &vox_opt, GaussianCloud &marginalized) {
    if (octo_state == 0 && layer >= 0) {
      if (!isexist || sw == nullptr) {
        return;
      }
      lock_guard<mutex> lock(mVox);
      vector<PointCluster> pcrs_world(wdsize);

      if (opt_state >= static_cast<int>(vox_opt.pcr_adds.size())) [[unlikely]] {
        throw std::logic_error("Invalid lidar-factor voxel index");
      }

      if (opt_state >= 0) {
        pcr_add = vox_opt.pcr_adds[opt_state];
        eig_value = vox_opt.eig_values[opt_state];
        eig_vector = vox_opt.eig_vectors[opt_state];
        opt_state = -1;

        for (int i = 0; i < mgsize; i++) {
          if (sw->pcrs_local[window_order[i]].N != 0) {
            pcrs_world[i].transform(sw->pcrs_local[window_order[i]], x_buf[i]);
          }
        }
      } else {
        pcr_add = pcr_fix;
        for (int i = 0; i < win_count; i++) {
          if (sw->pcrs_local[window_order[i]].N != 0) {
            pcrs_world[i].transform(sw->pcrs_local[window_order[i]], x_buf[i]);
            pcr_add += pcrs_world[i];
          }
        }

        if (plane.is_plane) {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
          eig_value = saes.eigenvalues();
          eig_vector = saes.eigenvectors();
        }
      }

      PointCluster marginalized_cluster;
      for (int i = 0; i < mgsize; ++i) {
        marginalized_cluster += pcrs_world[i];
      }
      if (marginalized_cluster.N > 0) {
        GaussianPoint gaussian = marginalized_cluster.to_gaussianpoint(voxel_key);
        gaussian.is_plane = plane.is_plane;
        marginalized.emplace_back(std::move(gaussian));
      }

      if (pcr_fix.N < max_points && plane.is_plane) {
        if (pcr_add.N - last_num >= 5 || last_num <= 10) {
          plane_update();
          last_num = pcr_add.N;
        }
      }

      if (pcr_fix.N < max_points) {
        for (int i = 0; i < mgsize; i++) {
          if (pcrs_world[i].N != 0) {
            pcr_fix += pcrs_world[i];
            for (pointVar pv : sw->points[window_order[i]]) {
              pv.pnt = x_buf[i].R * pv.pnt + x_buf[i].p;
              point_fix.push_back(pv);
            }
          }
        }
      } else {
        for (int i = 0; i < mgsize; i++) {
          if (pcrs_world[i].N != 0) {
            pcr_add -= pcrs_world[i];
          }
        }
        if (point_fix.size() != 0) {
          PointCloud().swap(point_fix);
        }
      }

      for (int i = 0; i < mgsize; i++) {
        if (sw->pcrs_local[window_order[i]].N != 0) {
          sw->pcrs_local[window_order[i]].clear();
          sw->points[window_order[i]].clear();
        }
      }
      isexist = pcr_fix.N < pcr_add.N;
    } else {
      isexist = false;
      for (int i = 0; i < 8; i++) {
        if (leaves[i] != nullptr) {
          leaves[i]->margi(win_count, mgsize, x_buf, vox_opt, marginalized);
          isexist = isexist || leaves[i]->isexist;
        }
      }
    }
  }

  void tras_opt(LidarFactor &vox_opt) {
    if (octo_state == 0) {
      if (layer >= 0 && isexist && plane.is_plane && sw != nullptr) {
        if (eig_value[0] / eig_value[1] > 0.12) {
          return;
        }

        double coe = 1;
        vector<PointCluster> pcrs(wdsize);
        for (int i = 0; i < wdsize; i++) {
          pcrs[i] = sw->pcrs_local[window_order[i]];
        }
        opt_state = vox_opt.plvec_voxels.size();
        vox_opt.push_voxel(pcrs, pcr_fix, coe, eig_value, eig_vector, pcr_add);
      }
    } else {
      for (int i = 0; i < 8; i++) {
        if (leaves[i] != nullptr) {
          leaves[i]->tras_opt(vox_opt);
        }
      }
    }
  }

  int match(Eigen::Vector3d &wld, Plane *&pla, double &max_prob, const pointVar &point, const Eigen::Matrix3d &rotation,
            const Eigen::Matrix<double, POSE_DOF, POSE_DOF> &pose_covariance,
            Eigen::Matrix<double, POSE_DOF, 1> &jacobian, double &sigma_d, OctoTree *&oc) {
    int flag = 0;
    if (octo_state == 0) {
      if (plane.is_plane) {
        float dis_to_plane = fabs(plane.normal.dot(wld - plane.center));
        float dis_to_center = (plane.center - wld).squaredNorm();
        float range_dis = (dis_to_center - dis_to_plane * dis_to_plane);
        if (range_dis <= 3 * 3 * plane.radius) {
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = wld - plane.center;
          J_nq.block<1, 3>(0, 3) = -plane.normal;
          double sigma_l = J_nq * plane.plane_var * J_nq.transpose();
          sigma_l += point.projectVariance(plane.normal, rotation, pose_covariance, jacobian);
          if (dis_to_plane < 3 * sqrt(sigma_l)) {
            {
              oc = this;
              sigma_d = sigma_l;
              pla = &plane;
            }
            flag = 1;
          }
        }
      }
    } else {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++) {
        if (wld[k] > voxel_center[k]) {
          xyz[k] = 1;
        }
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves[leafnum] != nullptr) {
        flag = leaves[leafnum]->match(wld, pla, max_prob, point, rotation, pose_covariance, jacobian, sigma_d, oc);
      }
    }
    return flag;
  }

  // Read-only query; caller holds the normal odometry/map lifecycle exclusion.
  // Traverse only intersecting nodes, and return moments even for nonplane leaves.
  void nearest_cluster(const Eigen::Vector3d &wld, double &distance_sq, const OctoTree *&best) const {
    const Eigen::Vector3d center(voxel_center[0], voxel_center[1], voxel_center[2]);
    const Eigen::Vector3d outside = ((wld - center).cwiseAbs().array() - 2.0 * quater_length).max(0.0);
    if (outside.squaredNorm() > distance_sq) return;
    if (octo_state != 0) {
      for (const auto &leaf : leaves) if (leaf) leaf->nearest_cluster(wld, distance_sq, best);
    } else if (pcr_add.N >= 6) {
      const double d = (wld - pcr_add.mean()).squaredNorm();
      if (std::isfinite(d) && d < distance_sq) { distance_sq = d; best = this; }
    }
  }

  bool inside(Eigen::Vector3d &wld) {
    double hl = quater_length * 2;
    return (wld[0] >= voxel_center[0] - hl && wld[0] <= voxel_center[0] + hl && wld[1] >= voxel_center[1] - hl && wld[1] <= voxel_center[1] + hl &&
            wld[2] >= voxel_center[2] - hl && wld[2] <= voxel_center[2] + hl);
  }

  void clear_slwd(vector<SlideWindow *> &sws) {
    if (octo_state != 0) {
      for (int i = 0; i < 8; i++) {
        if (leaves[i] != nullptr) {
          leaves[i]->clear_slwd(sws);
        }
      }
    }
    if (sw != nullptr) {
      sw->clear();
      sws.push_back(sw);
      sw = nullptr;
    }
  }
};

class VoxelMap {
 private:
  using TreeMap = std::unordered_map<VOXEL_LOC, unique_ptr<OctoTree>>;
  using ActiveMap = std::unordered_map<VOXEL_LOC, OctoTree *>;

  TreeMap local_voxel_map_;
  ActiveMap active_voxels_;
  vector<unique_ptr<OctoTree>> retired_trees_;
  vector<vector<SlideWindow *>> slide_window_pool_;
  int thread_num_;

 public:
  VoxelMap(const OdometryParameters &odometry, const LocalSubmapParameters &local_submap)
      : slide_window_pool_(std::max(1, local_submap.thread_num)), thread_num_(std::max(1, local_submap.thread_num)) {
    OctoTree::configure(odometry, local_submap);
  }

  ~VoxelMap() {
    clear();
    retired_trees_.clear();
    for (vector<SlideWindow *> &pool : slide_window_pool_) {
      for (SlideWindow *window : pool) {
        delete window;
      }
    }
  }

  // Owner-thread diagnostic after all per-frame workers have joined. Capacity
  // accounting excludes allocator metadata, hash nodes and other pipeline owners.
  struct MemoryUsage {
    std::size_t trees = 0, active_windows = 0, pooled_windows = 0;
    std::size_t tree_bytes = 0, window_bytes = 0, pooled_window_bytes = 0, point_capacity_bytes = 0, empty_slot_bytes = 0, leaf_point_bytes = 0;
  };
  MemoryUsage memoryUsage() const {
    MemoryUsage result;
    const auto window = [&](const SlideWindow &w, bool leaf) {
      result.window_bytes += sizeof(w) + w.points.capacity() * sizeof(PointCloud) + w.pcrs_local.capacity() * sizeof(PointCluster);
      for (const auto &points : w.points) {
        const auto bytes = points.capacity() * sizeof(pointVar);
        result.window_bytes += bytes;
        result.point_capacity_bytes += bytes;
        if (points.empty()) result.empty_slot_bytes += bytes;
        if (leaf) result.leaf_point_bytes += bytes;
      }
    };
    const auto visit = [&](const auto &self, const OctoTree &tree) -> void {
      ++result.trees;
      result.tree_bytes += sizeof(tree) + tree.point_fix.capacity() * sizeof(pointVar);
      if (tree.sw) { ++result.active_windows; window(*tree.sw, tree.layer >= OctoTree::max_layer); }
      for (const auto &child : tree.leaves) if (child) self(self, *child);
    };
    for (const auto &entry : local_voxel_map_) visit(visit, *entry.second);
    for (const auto &tree : retired_trees_) visit(visit, *tree);
    for (const auto &pool : slide_window_pool_) for (const auto *w : pool) { ++result.pooled_windows; result.pooled_window_bytes += w->allocatedBytes(); window(*w, false); }
    return result;
  }

  VoxelMap(const VoxelMap &) = delete;
  VoxelMap &operator=(const VoxelMap &) = delete;

  void cut_voxel(const PointCloudPtr &pvec, int win_count, int wdsize, vvec<double, 3> &pwld) {
    int plsize = pvec->size();
    for (int i = 0; i < plsize; i++) {
      pointVar &pv = (*pvec)[i];
      Eigen::Vector3d &pw = pwld[i];
      float loc[3];
      for (int j = 0; j < 3; j++) {
        loc[j] = pw[j] * OctoTree::voxel_size_inv;
        if (loc[j] < 0) {
          loc[j] -= 1;
        }
      }

      VOXEL_LOC position(loc[0], loc[1], loc[2]);
      auto iter = local_voxel_map_.find(position);
      if (iter != local_voxel_map_.end()) {
        OctoTree *tree = iter->second.get();
        tree->allocate(win_count, pv, pw, slide_window_pool_[0]);
        tree->isexist = true;
        active_voxels_.try_emplace(position, tree);
      } else {
        auto owner = make_unique<OctoTree>(0, wdsize, position);
        OctoTree *ot = owner.get();
        ot->voxel_center[0] = (0.5 + position.x) * OctoTree::voxel_size;
        ot->voxel_center[1] = (0.5 + position.y) * OctoTree::voxel_size;
        ot->voxel_center[2] = (0.5 + position.z) * OctoTree::voxel_size;
        ot->quater_length = OctoTree::voxel_size / 4.0;
        ot->allocate(win_count, pv, pw, slide_window_pool_[0]);
        local_voxel_map_.emplace(position, std::move(owner));
        active_voxels_.emplace(position, ot);
      }
    }
  }

  void cut_voxel_multi(const PointCloudPtr &pvec, int win_count, int wdsize, vvec<double, 3> &pwld) {
    unordered_map<OctoTree *, vector<int>> map_pvec;
    int plsize = pvec->size();
    for (int i = 0; i < plsize; i++) {
      pointVar &pv = (*pvec)[i];
      Eigen::Vector3d &pw = pwld[i];
      float loc[3];
      for (int j = 0; j < 3; j++) {
        loc[j] = pw[j] * OctoTree::voxel_size_inv;
        if (loc[j] < 0) {
          loc[j] -= 1;
        }
      }

      VOXEL_LOC position(loc[0], loc[1], loc[2]);
      auto iter = local_voxel_map_.find(position);
      OctoTree *ot = nullptr;
      if (iter != local_voxel_map_.end()) {
        ot = iter->second.get();
        ot->isexist = true;
        active_voxels_.try_emplace(position, ot);
      } else {
        auto owner = make_unique<OctoTree>(0, wdsize, position);
        ot = owner.get();
        ot->voxel_center[0] = (0.5 + position.x) * OctoTree::voxel_size;
        ot->voxel_center[1] = (0.5 + position.y) * OctoTree::voxel_size;
        ot->voxel_center[2] = (0.5 + position.z) * OctoTree::voxel_size;
        ot->quater_length = OctoTree::voxel_size / 4.0;
        local_voxel_map_.emplace(position, std::move(owner));
        active_voxels_.emplace(position, ot);
      }

      map_pvec[ot].push_back(i);
    }

    vector<pair<OctoTree *const, vector<int>> *> octs;
    octs.reserve(map_pvec.size());
    for (auto iter = map_pvec.begin(); iter != map_pvec.end(); iter++) {
      octs.push_back(&(*iter));
    }

    int thd_num = thread_num_;
    int g_size = octs.size();
    if (g_size < thd_num) {
      for (auto &entry : map_pvec) {
        for (int index : entry.second) {
          entry.first->allocate(win_count, (*pvec)[index], pwld[index], slide_window_pool_[0]);
        }
      }
      return;
    }
    int swsize = slide_window_pool_[0].size() / thd_num;
    for (int i = 1; i < thd_num; i++) {
      slide_window_pool_[i].insert(slide_window_pool_[i].end(), slide_window_pool_[0].end() - swsize, slide_window_pool_[0].end());
      slide_window_pool_[0].erase(slide_window_pool_[0].end() - swsize, slide_window_pool_[0].end());
    }

    const int part = (g_size + thd_num - 1) / thd_num;

    auto allocate_range = [&](int head, int tail, vector<SlideWindow *> &pool) {
      for (int j = head; j < tail; ++j) {
        for (int index : octs[j]->second) {
          octs[j]->first->allocate(win_count, (*pvec)[index], pwld[index], pool);
        }
      }
    };

    vector<thread> workers;
    workers.reserve(thd_num - 1);
    for (int i = 1; i < thd_num; i++) {
      const int head = std::min(i * part, g_size);
      const int tail = std::min(head + part, g_size);
      workers.emplace_back(allocate_range, head, tail, ref(slide_window_pool_[i]));
    }
    allocate_range(0, std::min(part, g_size), slide_window_pool_[0]);
    for (thread &worker : workers) {
      worker.join();
    }
  }

  void cut_voxel(PointCloud &pvec, int wdsize, double jour) {
    for (pointVar &pv : pvec) {
      float loc[3];
      for (int j = 0; j < 3; j++) {
        loc[j] = pv.pnt[j] * OctoTree::voxel_size_inv;
        if (loc[j] < 0) {
          loc[j] -= 1;
        }
      }

      VOXEL_LOC position(loc[0], loc[1], loc[2]);
      auto iter = local_voxel_map_.find(position);
      if (iter != local_voxel_map_.end()) {
        iter->second->allocate_fix(pv);
      } else {
        auto owner = make_unique<OctoTree>(0, wdsize, position);
        OctoTree *ot = owner.get();
        ot->push_fix_novar(pv);
        ot->voxel_center[0] = (0.5 + position.x) * OctoTree::voxel_size;
        ot->voxel_center[1] = (0.5 + position.y) * OctoTree::voxel_size;
        ot->voxel_center[2] = (0.5 + position.z) * OctoTree::voxel_size;
        ot->quater_length = OctoTree::voxel_size / 4.0;
        ot->jour = jour;
        local_voxel_map_.emplace(position, std::move(owner));
      }
    }
  }

  const OctoTree *nearest_cluster(const Eigen::Vector3d &wld, double max_distance) const {
    if (!wld.allFinite() || !std::isfinite(max_distance) || max_distance <= 0 ||
        max_distance > OctoTree::voxel_size || wld.cwiseAbs().maxCoeff() > 1e6) return nullptr;
    // Same indexing convention as insertion, including exact negative boundaries.
    int64_t key[3];
    for (int i = 0; i < 3; ++i) {
      float v = wld[i] * OctoTree::voxel_size_inv;
      if (v < 0) v -= 1;
      key[i] = static_cast<int64_t>(v);
    }
    double d = max_distance * max_distance;
    const OctoTree *best = nullptr;
    for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z) {
      const auto found = local_voxel_map_.find(VOXEL_LOC(key[0]+x,key[1]+y,key[2]+z));
      if (found != local_voxel_map_.end()) found->second->nearest_cluster(wld, d, best);
    }
    return best;
  }

  int match(Eigen::Vector3d &wld, Plane *&pla, const pointVar &point, const Eigen::Matrix3d &rotation,
            const Eigen::Matrix<double, POSE_DOF, POSE_DOF> &pose_covariance,
            Eigen::Matrix<double, POSE_DOF, 1> &jacobian, double &sigma_d, OctoTree *&oc) {
    int flag = 0;

    float loc[3];
    for (int j = 0; j < 3; j++) {
      loc[j] = wld[j] * OctoTree::voxel_size_inv;
      if (loc[j] < 0) {
        loc[j] -= 1;
      }
    }
    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = local_voxel_map_.find(position);
    if (iter != local_voxel_map_.end()) {
      double max_prob = 0;
      flag = iter->second->match(wld, pla, max_prob, point, rotation, pose_covariance, jacobian, sigma_d, oc);
      if (flag && pla == nullptr) {
        spdlog::warn("pla null max_prob: {} {} {} {}", max_prob, iter->first.x, iter->first.y, iter->first.z);
      }
    }

    return flag;
  }

  void recut(int win_count, vector<StateGroup> &states, LidarFactor &factor, double eigen_threshold, const vector<double> &plane_thresholds) {
    for (auto &entry : active_voxels_) {
      entry.second->recut(win_count, states, slide_window_pool_[0], eigen_threshold, plane_thresholds);
    }
    for (auto &entry : active_voxels_) {
      entry.second->tras_opt(factor);
    }
  }

  void recut_multi(int win_count, vector<StateGroup> &states, LidarFactor &factor, double eigen_threshold, const vector<double> &plane_thresholds) {
    vector<OctoTree *> trees;
    trees.reserve(active_voxels_.size());
    for (auto &entry : active_voxels_) {
      trees.push_back(entry.second);
    }

    const int tree_count = static_cast<int>(trees.size());
    const int worker_count = std::min(thread_num_, tree_count);
    if (worker_count <= 1) {
      recut(win_count, states, factor, eigen_threshold, plane_thresholds);
      return;
    }

    const int part = (tree_count + worker_count - 1) / worker_count;

    auto recut_range = [&](int head, int tail, vector<SlideWindow *> &pool) {
      for (int i = head; i < tail; ++i) {
        trees[i]->recut(win_count, states, pool, eigen_threshold, plane_thresholds);
      }
    };

    vector<thread> workers;
    workers.reserve(worker_count - 1);
    for (int i = 1; i < worker_count; ++i) {
      const int head = std::min(i * part, tree_count);
      const int tail = std::min(head + part, tree_count);
      workers.emplace_back(recut_range, head, tail, ref(slide_window_pool_[i]));
    }
    recut_range(0, std::min(part, tree_count), slide_window_pool_[0]);
    for (thread &worker : workers) {
      worker.join();
    }

    for (int i = 1; i < worker_count; ++i) {
      slide_window_pool_[0].insert(slide_window_pool_[0].end(), slide_window_pool_[i].begin(), slide_window_pool_[i].end());
      slide_window_pool_[i].clear();
    }
    for (OctoTree *tree : trees) {
      tree->tras_opt(factor);
    }
  }

  GaussianCloud marginalize(double journey, int win_count, int margin_size, vector<StateGroup> &states, LidarFactor &factor) {
    vector<OctoTree *> trees;
    trees.reserve(active_voxels_.size());
    for (auto &entry : active_voxels_) {
      entry.second->jour = journey;
      trees.push_back(entry.second);
    }

    const int tree_count = static_cast<int>(trees.size());
    const int worker_count = std::min(thread_num_, tree_count);
    vector<GaussianCloud> worker_clouds(std::max(1, worker_count));
    if (worker_count <= 1) {
      for (OctoTree *tree : trees) {
        tree->margi(win_count, margin_size, states, factor, worker_clouds[0]);
      }
    } else {
      const int part = (tree_count + worker_count - 1) / worker_count;
      auto margi_range = [&](int head, int tail, GaussianCloud &cloud) {
        for (int i = head; i < tail; ++i) {
          trees[i]->margi(win_count, margin_size, states, factor, cloud);
        }
      };

      vector<thread> workers;
      workers.reserve(worker_count - 1);
      for (int i = 1; i < worker_count; ++i) {
        const int head = std::min(i * part, tree_count);
        workers.emplace_back(margi_range, head, std::min(head + part, tree_count), ref(worker_clouds[i]));
      }
      margi_range(0, std::min(part, tree_count), worker_clouds[0]);
      for (thread &worker : workers) {
        worker.join();
      }
    }

    for (auto iter = active_voxels_.begin(); iter != active_voxels_.end();) {
      if (iter->second->isexist) {
        ++iter;
      } else {
        iter->second->clear_slwd(slide_window_pool_[0]);
        iter = active_voxels_.erase(iter);
      }
    }
    OctoTree::advance_window_order(margin_size);

    size_t gaussian_count = 0;
    for (const GaussianCloud &cloud : worker_clouds) {
      gaussian_count += cloud.size();
    }
    GaussianCloud marginalized;
    marginalized.reserve(gaussian_count);
    for (GaussianCloud &cloud : worker_clouds) {
      marginalized.insert(marginalized.end(), std::make_move_iterator(cloud.begin()), std::make_move_iterator(cloud.end()));
    }
    return marginalized;
  }

  void clear() {
    active_voxels_.clear();
    for (auto &entry : local_voxel_map_) {
      entry.second->clear_slwd(slide_window_pool_[0]);
    }
    local_voxel_map_.clear();
    OctoTree::reset_window_order();
  }

  void reset() {
    active_voxels_.clear();
    retired_trees_.reserve(retired_trees_.size() + local_voxel_map_.size());
    for (auto &entry : local_voxel_map_) {
      entry.second->clear_slwd(slide_window_pool_[0]);
      retired_trees_.push_back(std::move(entry.second));
    }
    local_voxel_map_.clear();
    OctoTree::reset_window_order();
  }

  void prune(double journey, double max_distance) {
    for (auto iter = local_voxel_map_.begin(); iter != local_voxel_map_.end();) {
      if (journey - iter->second->jour < max_distance) {
        ++iter;
        continue;
      }
      active_voxels_.erase(iter->first);
      iter->second->clear_slwd(slide_window_pool_[0]);
      retired_trees_.push_back(std::move(iter->second));
      iter = local_voxel_map_.erase(iter);
    }
  }

  size_t release_retired(size_t max_count) {
    const size_t count = std::min(max_count, retired_trees_.size());
    for (size_t i = 0; i < count; ++i) {
      retired_trees_.pop_back();
    }
    return count;
  }

  // Only empty reusable windows are evicted. Active windows/observations are
  // untouched. One byte budget covers all worker pools, not one budget per worker.
  static constexpr std::size_t window_pool_byte_budget = 64U * 1024 * 1024;
  size_t trim_window_pool(size_t max_retained, size_t max_release) {
    std::size_t bytes = 0;
    for (const auto &pool : slide_window_pool_) for (const auto *window : pool) bytes += window->allocatedBytes();
    size_t released = 0;
    for (vector<SlideWindow *> &pool : slide_window_pool_) {
      while (!pool.empty() && released < max_release && (pool.size() > max_retained || bytes > window_pool_byte_budget)) {
        bytes -= pool.back()->allocatedBytes();
        delete pool.back();
        pool.pop_back();
        ++released;
      }
      if (released == max_release) break;
    }
    return released;
  }

  size_t size() const { return local_voxel_map_.size(); }
  size_t active_size() const { return active_voxels_.size(); }
};

}  // namespace sapphire
