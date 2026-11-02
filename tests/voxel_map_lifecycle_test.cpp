#include "maplayer/local_map/voxel_map.hpp"
#include <atomic>
#include <cassert>
#include <iostream>
using namespace sapphire;
namespace sapphire {
struct VoxelMapTestAccess {
  static void fixed_lru() {
    OdometryParameters o;
    o.fixed_map_budget_bytes = 2 * (sizeof(VoxelMap::Root) + sizeof(OctoTree));
    LocalSubmapParameters l;
    tools::ParallelExecutor<> executor(1);
    VoxelMap map(o, l, executor);
    const auto add = [&](int x) {
      VOXEL_LOC key(x, 0, 0);
      map.local_voxel_map_.emplace(key, std::make_unique<OctoTree>(map.layout_, 0, 10, key));
      map.cacheFixed(key);
    };
    add(0); add(1);
    assert(map.trimFixed() == 0);
    map.cacheFixed(VOXEL_LOC(0, 0, 0)); // Re-observed fixed history is most recent.
    add(2);
    assert(map.trimFixed() == 1);
    assert(!map.local_voxel_map_.count(VOXEL_LOC(1, 0, 0)));
    auto &pinned = map.local_voxel_map_.at(VOXEL_LOC(0, 0, 0));
    map.pin(pinned);
    map.active_voxels_.emplace(VOXEL_LOC(0, 0, 0), pinned.tree.get());
    pinned.tree->point_fix.reserve(10000); // Active bytes may exceed cache budget.
    add(3); add(4);
    assert(map.trimFixed() == 1 && map.active_size() == 1);
    assert(map.local_voxel_map_.count(VOXEL_LOC(0, 0, 0)));
    // The identical root becomes evictable only after all active support leaves.
    map.active_voxels_.clear();
    map.cacheFixed(VOXEL_LOC(0, 0, 0));
    assert(map.trimFixed() == 3 && map.fixedBytes() == 0 && map.size() == 0);
    add(5); map.reset();
    assert(map.fixedSize() == 0 && map.fixedBytes() == 0);
    assert(map.release_retired(100) == 1);
    add(6); map.clear();
    assert(map.fixedSize() == 0 && map.fixedBytes() == 0);
  }
  static std::vector<double> snapshot(const VoxelMap &map) {
    std::vector<double> out;
    auto matrix = [&](const auto &m) {
      for (int i = 0; i < m.size(); ++i)
        out.push_back(m.data()[i]);
    };
    auto cluster = [&](const PointCluster &c) {
      out.push_back(c.N);
      matrix(c.v);
      matrix(c.P);
    };
    auto visit = [&](const auto &self, const OctoTree &t) -> void {
      out.push_back(t.layer);
      out.push_back(t.octo_state);
      out.push_back(t.isexist);
      cluster(t.pcr_add);
      cluster(t.pcr_fix);
      out.push_back(t.plane.is_plane);
      if (t.plane.is_plane) {
        matrix(t.eig_value);
        matrix(t.eig_vector);
        matrix(t.plane.center);
        matrix(t.plane.normal);
        matrix(t.plane.plane_var);
      }
      out.push_back(t.point_fix.size());
      for (const auto &v : t.point_fix) {
        matrix(v.pnt);
        matrix(v.var);
      }
      if (t.sw)
        for (const auto &c : t.sw->pcrs_local)
          cluster(c);
      for (const auto &child : t.leaves)
        if (child)
          self(self, *child);
    };
    for (const auto &entry : map.local_voxel_map_)
      visit(visit, *entry.second.tree);
    return out;
  }
  static void worker_failure(VoxelMap &map, bool background) {
    for (int bad = 0; bad < 2; ++bad) {
      const VOXEL_LOC key(bad, 0, 0);
      auto t = std::make_unique<OctoTree>(map.layout_, 0, 4, key);
      t->voxel_center[0] = t->voxel_center[1] = t->voxel_center[2] = .5;
      t->quater_length = .25;
      for (int i = 0; i < 120; ++i) {
        pointVar v;
        v.pnt = {.05 + .07 * (i % 11), .07 + .06 * (i % 13),
                 .09 + .05 * (i % 17)};
        v.var = Eigen::Matrix3d::Identity() * .001;
        t->push(0, v, v.pnt, map.slide_window_pool_[0]);
      }
      map.active_voxels_.emplace(key, t.get());
      map.local_voxel_map_.emplace(key, std::move(t));
    }
    // Inject overflow into a selected partition without assuming hash order.
    // Two roots / two workers gives one root to the owner and one to a worker.
    auto victim = map.active_voxels_.begin();
    if (background) ++victim;
    victim->second->voxel_key = VOXEL_LOC(std::numeric_limits<int64_t>::max(), 0, 0);
    std::vector<StateGroup> states(4);
    LidarFactor factor(4);
    bool threw = false;
    try {
      map.recut_multi(1, states, 0, {1, 1, 1, 1});
    } catch (const std::overflow_error &) {
      threw = true;
    }
    assert(threw);
    // All partition tasks must have joined before cleanup or another dispatch.
    std::atomic<int> jobs{0};
    map.executor_.parallel_for(
        2, 1, [&](size_t, size_t begin, size_t end) { jobs += end - begin; });
    assert(jobs == 2);
    map.clear();
    map.trim_window_pool(0, 10000);
    assert(map.size() == 0);
  }
  static void accounting(VoxelMap &map) {
    auto &pool = map.slide_window_pool_[0];
    for (int i = 0; i < 7; ++i)
      pool.push_back(new SlideWindow(10));
    map.clear(); // Public lifecycle marks returned storage for recount.
    assert(map.trim_window_pool(100, 100) == 0);
    for (int i = 0; i < 100; ++i)
      assert(map.trim_window_pool(100, 100) == 0);
    assert(map.trim_window_pool(2, 3) == 3);
    assert(map.memoryUsage().pooled_windows == 4);
    assert(map.trim_window_pool(2, 100) ==
           2); // Clean cache still honors new limits.
    assert(map.trim_window_pool(0, 100) == 2);
    // Reserve empty slot capacities: this checks the byte budget independently
    // of count budget without adding points or touching uninitialized storage.
    auto *large = new SlideWindow(10);
    large->points[0].reserve(
        VoxelMap::window_pool_byte_budget / sizeof(pointVar) + 1);
    pool.push_back(large);
    map.clear();
    assert(map.trim_window_pool(100, 0) == 0);
    assert(map.memoryUsage().pooled_window_bytes >
           VoxelMap::window_pool_byte_budget);
    assert(map.trim_window_pool(100, 1) == 1);
    assert(map.memoryUsage().pooled_window_bytes == 0);
    assert(map.trim_window_pool(0, 100) == 0);
  }
};
} // namespace sapphire
void lifecycle() {
  VoxelMapTestAccess::fixed_lru();
  OdometryParameters o;
  LocalSubmapParameters l;
  l.win_size = 4;
  l.thread_num = 2;
  tools::ParallelExecutor<> executor(l.thread_num);
  VoxelMap map(o, l, executor);
  VoxelMapTestAccess::accounting(map);
  for (int round = 0; round < 3; ++round) {
    std::vector<StateGroup> states(4);
    LidarFactor factor(4);
    auto points = std::make_shared<PointCloud>();
    vvec<double, 3> world;
    for (int i = 0; i < 120; ++i) {
      pointVar p;
      p.pnt = {.1 + .01 * (i % 10), .1 + .01 * (i / 10), .2};
      p.var = Eigen::Matrix3d::Identity() * .001;
      points->push_back(p);
      world.push_back(p.pnt);
    }
    map.cut_voxel_multi(points, 0, 4, world);
    map.recut_multi(1, states, .01, {1, 1, 1, 1});
    map.trim_window_pool(100, 100);
    map.marginalize(round, 1, 1, states);
    auto before = map.memoryUsage().pooled_windows;
    auto released = map.trim_window_pool(0, 1);
    assert(released <= 1 &&
           map.memoryUsage().pooled_windows + released == before);
    map.trimFixed();
    map.trim_window_pool(0, 1000);
    map.reset();
    map.trim_window_pool(0, 1000);
    while (map.release_retired(1)) {
    };
    map.clear();
    map.trim_window_pool(0, 1000);
    assert(map.memoryUsage().pooled_window_bytes == 0 && map.size() == 0);
  }
}
void marginal_equivalence() {
  for (int margin : {0, 1, 2}) {
    auto run = [&](bool emit) {
      OdometryParameters o;
      LocalSubmapParameters l;
      l.win_size = 4;
      l.thread_num = 2;
      tools::ParallelExecutor<> executor(l.thread_num);
      VoxelMap map(o, l, executor);
      std::vector<StateGroup> states(4);
      LidarFactor factor(4);
      for (int scan = 0; scan < 4; ++scan) {
        states[scan].R =
            Eigen::AngleAxisd(.07 * scan, Eigen::Vector3d(1, 2, 3).normalized())
                .toRotationMatrix();
        states[scan].p = {.03 * scan, -.02 * scan, .01 * scan};
        auto points = std::make_shared<PointCloud>();
        vvec<double, 3> world;
        for (int group = 0; group < 8; ++group)
          for (int i = 0; i < 100; ++i) {
            Eigen::Vector3d w(2 * group + .12 + .03 * (i % 10),
                              .13 + .03 * (i / 10), .21);
            pointVar v;
            v.pnt = states[scan].R.transpose() * (w - states[scan].p);
            v.var = Eigen::Vector3d(.001, .002, .003).asDiagonal();
            points->push_back(v);
            world.push_back(w);
          }
        map.cut_voxel_multi(points, scan, 4, world);
        factor.clear();
        map.recut_multi(scan + 1, states, .01, {1, 1, 1, 1});
      }
      map.collect_factors(factor);
      assert(!factor.plvec_voxels.empty());
      auto output = map.marginalize(1, 4, margin, states, emit);
      if (emit && margin > 0)
        assert(!output.empty());
      if (!emit)
        assert(output.empty());
      return VoxelMapTestAccess::snapshot(map);
    };
    const auto a = run(true), b = run(false);
    assert(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i)
      assert(a[i] == b[i]);
  }
  std::cout << "PASS enabled/disabled evidence preserves identical frontend "
               "state for zero/one/two-frame margins\n";
}
void inherited_history_plane_initialization() {
  // Cover the history-admission boundary in every child octant. Splitting a
  // saturated parent must initialize the child's plane before freezing updates.
  for (int count : {99, 100, 101, 120}) {
    for (int octant = 0; octant < 8; ++octant) {
      OdometryParameters odometry;
      LocalSubmapParameters local;
      local.win_size = 4;
      local.max_layer = 2;
      OctoTree::Layout layout(local);
      std::vector<SlideWindow *> pool;
      {
        OctoTree tree(layout, 0, 4, VOXEL_LOC(0, 0, 0));
        for (double &center : tree.voxel_center) center = .5;
        tree.quater_length = .25;
        std::vector<StateGroup> states(4);
        std::vector<PointCluster> scratch(4);
        LidarFactor factors(4);
        GaussianCloud ignored;
        const std::vector<double> thresholds(4, .25);
        auto push = [&](double x, double y, double z) {
          pointVar point;
          point.pnt = {x, y, z};
          for (int axis = 0; axis < 3; ++axis)
            if (octant & (4 >> axis)) point.pnt[axis] = 1 - point.pnt[axis];
          point.var = Eigen::Matrix3d::Identity() * .001;
          tree.allocate(0, point, point.pnt, pool);
        };
        auto marginalize = [&]() {
          tree.margi(1, 1, states, ignored, scratch, false);
        };
        const int width = static_cast<int>(std::ceil(std::sqrt(count)));
        for (int i = 0; i < count; ++i)
          push(.05 + .32 * (i % width) / (width - 1),
               .05 + .32 * (i / width) / (width - 1), .2);
        tree.recut(1, states, pool, .0025, thresholds);
        marginalize();
        assert(tree.octo_state == 0 && tree.plane.is_plane);
        assert(tree.pcr_fix.N == count);

        // Nonplanar observations in the opposite octant force subdivision.
        for (int x = 0; x < 4; ++x)
          for (int y = 0; y < 4; ++y)
            for (int z = 0; z < 4; ++z)
              push(.56 + .1 * x, .56 + .1 * y, .56 + .1 * z);
        for (int i = 0; i < 12; ++i) push(.055 + .025 * i, .15, .2);
        tree.recut(1, states, pool, .0025, thresholds);
        assert(tree.octo_state == 1 && tree.leaves[octant]);
        auto &child = *tree.leaves[octant];
        assert(child.pcr_fix.N == count && child.last_num == 0);
        const Eigen::Vector3d expected_center = child.pcr_add.v / child.pcr_add.N;
        marginalize();
        assert(child.octo_state == 0 && child.plane.is_plane);
        assert(child.last_num == count + 12);
        assert(std::abs(child.plane.normal.norm() - 1) < 1e-10);
        assert(std::abs(child.plane.normal.z()) > 1 - 1e-10);
        assert(child.plane.center.isApprox(expected_center, 1e-10));
        assert(child.plane.plane_var.allFinite());
        assert(child.plane.plane_var.trace() > 0);
        assert(child.pcr_fix.N == (count < OctoTree::max_points ? count + 12 : count));

        // Initialization is a one-time exception, not renewed admission or
        // repeated updates after the existing fixed-history cap is reached.
        const auto plane = child.plane;
        const int last_num = child.last_num;
        const int fixed_count = child.pcr_fix.N;
        for (int i = 0; i < 12; ++i) push(.055 + .025 * i, .15, .201);
        tree.recut(1, states, pool, .0025, thresholds);
        marginalize();
        assert(child.pcr_fix.N == fixed_count && child.point_fix.empty());
        assert(child.last_num == last_num);
        assert(child.plane.center == plane.center);
        assert(child.plane.normal == plane.normal);
        assert(child.plane.plane_var == plane.plane_var);
        tree.clear_slwd(pool);
      }
      for (auto *window : pool) delete window;
    }
  }
  std::cout << "PASS inherited history initializes child planes at saturation; "
               "subsequent admission and frozen planes remain unchanged\n";
}

void nonplanar_geometry_retention() {
  // A plane-factor rejection must not discard the geometry used by later scans.
  // Compare leaf sufficient statistics, not only vector sizes: deepest leaves
  // intentionally retain moments instead of raw per-point payloads.
  for (int depth : {0, 2}) {
    OdometryParameters odometry;
    LocalSubmapParameters local;
    local.win_size = 3;
    local.max_layer = depth;
    OctoTree::Layout layout(local);
    layout.reset_window_order();
    std::vector<SlideWindow *> pool;
    {
      OctoTree tree(layout, 0, 3, VOXEL_LOC(0, 0, 0));
      for (double &center : tree.voxel_center) center = .5;
      tree.quater_length = .25;
      PointCluster expected;
      for (double x : {.1, .3, .6, .8})
        for (double y : {.1, .3, .6, .8})
          for (double z : {.1, .3, .6, .8}) {
            pointVar p;
            p.pnt = {x, y, z};
            p.var = Eigen::Matrix3d::Identity() * .001;
            tree.allocate(0, p, p.pnt, pool);
            expected.push(p.pnt);
          }
      std::vector<StateGroup> states(3);
      tree.recut(1, states, pool, .0025, {.25, .25, .25, .25});
      LidarFactor factors(3);
      tree.tras_opt(factors);
      assert(factors.coeffs.empty());
      const auto collect = [&](const auto &self, const OctoTree &node,
                               bool fixed, PointCluster &sum) -> void {
        if (node.octo_state == 0) sum += fixed ? node.pcr_fix : node.pcr_add;
        else for (const auto &child : node.leaves) if (child) self(self, *child, fixed, sum);
      };
      const auto verify = [&](bool fixed) {
        PointCluster sum;
        collect(collect, tree, fixed, sum);
        assert(sum.N == expected.N);
        assert(sum.v.isApprox(expected.v, 1e-12));
        assert(sum.P.isApprox(expected.P, 1e-12));
      };
      verify(false);
      assert((depth == 0 && tree.octo_state == 0 && !tree.plane.is_plane) ||
             (depth == 2 && tree.octo_state == 1));
      if (depth == 0) assert(tree.sw && tree.sw->points[0].empty() && tree.sw->pcrs_local[0].N == 64);
      GaussianCloud output;
      std::vector<PointCluster> scratch(3);
      tree.margi(1, 1, states, output, scratch, true);
      verify(false);
      verify(true);
      size_t exported = 0;
      for (const auto &g : output) { assert(!g.is_plane); exported += g.N; }
      assert(exported == 64);
      tree.clear_slwd(pool);
    }
    for (auto *window : pool) delete window;
  }
  std::cout << "PASS nonplanar geometry: 64/64 points retained through recursive "
               "split, maximum depth, marginalization and nonplane export; zero BA plane factors\n";
}

void late_surface_recursive_split() {
  OdometryParameters odometry;
  LocalSubmapParameters local;
  local.win_size = 3;
  local.max_layer = 2;
  OctoTree::Layout layout(local);
  layout.reset_window_order();
  std::vector<SlideWindow *> pool;
  {
    OctoTree tree(layout, 0, 3, VOXEL_LOC(0, 0, 0));
    for (double &center : tree.voxel_center) center = .5;
    tree.quater_length = .25;
    std::vector<StateGroup> states(3);
    for (int frame = 0; frame < 2; ++frame) {
      for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
        pointVar p;
        p.pnt = frame == 0 ? Eigen::Vector3d(.05 + .13*i, .05 + .13*j, .15)
                           : Eigen::Vector3d(.7, .05 + .13*i, .55 + .13*j);
        p.var = Eigen::Matrix3d::Identity() * .001;
        tree.allocate(frame, p, p.pnt, pool);
      }
      tree.recut(frame + 1, states, pool, .0025, {.25, .25, .25, .25});
      if (frame == 0) assert(tree.octo_state == 0 && tree.plane.is_plane && tree.pcr_add.N == 16);
    }
    int points = 0, planes = 0;
    bool floor = false, wall = false;
    const auto visit = [&](const auto &self, const OctoTree &node) -> void {
      if (node.octo_state == 0) {
        points += node.pcr_add.N;
        if (node.plane.is_plane) {
          ++planes;
          floor |= std::abs(node.eig_vector.col(0).z()) > 1 - 1e-10;
          wall |= std::abs(node.eig_vector.col(0).x()) > 1 - 1e-10;
        }
      } else for (const auto &child : node.leaves) if (child) self(self, *child);
    };
    visit(visit, tree);
    assert(tree.octo_state == 1 && points == 32 && planes == 2 && floor && wall);
    tree.clear_slwd(pool);
  }
  for (auto *window : pool) delete window;
  std::cout << "PASS late perpendicular surface: 32/32 points reach recursive "
               "children and both geometric planes survive\n";
}

int main() {
  nonplanar_geometry_retention();
  late_surface_recursive_split();
  inherited_history_plane_initialization();
  lifecycle();
  {
    OdometryParameters o;
    LocalSubmapParameters l;
    l.thread_num = 2;
    tools::ParallelExecutor<> executor(1);
    bool rejected = false;
    try {
      VoxelMap map(o, l, executor);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
  }
  for (bool background : {false, true}) {
    OdometryParameters o;
    LocalSubmapParameters l;
    l.win_size = 4;
    l.thread_num = 2;
    tools::ParallelExecutor<> executor(2);
    VoxelMap map(o, l, executor);
    VoxelMapTestAccess::worker_failure(map, background);
  }
  std::cout << "PASS owner/background map partition exception joins before cleanup and next "
               "dispatch\n";
  marginal_equivalence();
  std::cout << "PASS pool budget, clean-cache limits, partial trim, borrowing, "
               "recut, marginalization, prune/reset/clear lifecycle\n";
}
