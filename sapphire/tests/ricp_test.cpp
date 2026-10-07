#include "frontend/ricp/ricp.hpp"
#include "frontend/eskf/eskf.hpp"
#include <cassert>
#include <iostream>

using namespace sapphire;

int main() {
  const Eigen::Matrix3d R = lie::SO3d::exp(Eigen::Vector3d(.2,-.1,.3)).R();
  OdometryParameters odom; LocalSubmapParameters local; local.thread_num = 1;
  VoxelMap map(odom,local);
  PointCloud world, source;
  for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z)
    for (int a = -1; a <= 1; ++a) for (int b = -1; b <= 1; ++b) for (int c = -1; c <= 1; ++c) {
      pointVar p; p.pnt = Eigen::Vector3d(.5+1.2*x+.04*a,.5+1.3*y+.04*b,.5+1.4*z+.04*c);
      p.var = Eigen::Matrix3d::Identity() * .0001; world.push_back(p);
      p.pnt = R.transpose()*p.pnt; source.push_back(p);
    }
  map.cut_voxel(world,0,0);
  const auto count = map.size();
  const auto *node = map.nearest_cluster(Eigen::Vector3d(-.7,-.8,-.9),.2);
  assert(node && !node->plane.is_plane && node->pcr_add.N == 27);
  assert(!map.nearest_cluster(Eigen::Vector3d(50,50,50),.5));
  assert(!map.nearest_cluster(Eigen::Vector3d::Zero(),std::numeric_limits<double>::quiet_NaN()));
  StateGroup state; state.R = R*lie::SO3d::exp(Eigen::Vector3d(.01,-.012,.007)).R();
  state.p << .03,-.02,.015; state.v << .1,.2,.3;
  state.cov.setIdentity(); state.cov *= .02;
  state.cov.block<3,3>(6,3) = state.cov.block<3,3>(3,6) = .005*Eigen::Matrix3d::Identity();
  state.cov.block<3,3>(12,3) = state.cov.block<3,3>(3,12) = .002*Eigen::Matrix3d::Identity();
  const auto prior = state;
  ESKF filter; ESKF::Rejection reason;
  assert(!filter.observe_voxelmap(state,source,map,&reason));
  assert(reason == ESKF::Rejection::no_matches);
  assert((state.cov.array()==prior.cov.array()).all() && state.p==prior.p);
  GicpFallbackParameters cfg;
  auto unfinished = prior;
  auto one_iteration = cfg; one_iteration.max_iterations = 1;
  const auto rejected_iteration = Ricp(one_iteration).observe(unfinished,source,map);
  assert(!rejected_iteration.accepted && std::string(rejected_iteration.reason)=="not_converged");
  assert(unfinished.p==prior.p && unfinished.R==prior.R && unfinished.v==prior.v && unfinished.bg==prior.bg && unfinished.ba==prior.ba);
  assert((unfinished.cov.array()==prior.cov.array()).all());
  const auto result = Ricp(cfg).observe(state,source,map);
  std::cout << "accepted=" << result.accepted << " reason=" << result.reason << " samples=" << result.samples << " matches=" << result.matches << " correction=" << result.correction << " error=" << state.p.norm() << '\n';
  assert(result.accepted);
  assert(state.p.norm() < prior.p.norm()*.8);
  assert((state.v-prior.v-.25*(state.p-prior.p)).norm() < 1e-10);
  assert((state.ba-prior.ba-.1*(state.p-prior.p)).norm() < 1e-10);
  assert(lie::SO3d::log(lie::SO3d(R.transpose()*state.R)).norm() < .017*.8);
  assert(state.cov.llt().info() == Eigen::Success && state.cov.trace() < prior.cov.trace());
  assert(map.size()==count && node->pcr_add.N==27 && !node->plane.is_plane);
  // Independent frame-equivariance oracle: changing world coordinates must not
  // change the physical update or covariance (library translation is body-frame,
  // ESKF translation is world-frame). Axis permutation preserves voxel grouping.
  Eigen::Matrix3d Q; Q << 0,-1,0, 1,0,0, 0,0,1;
  const Eigen::Vector3d offset(5,7,9);
  PointCloud transformed_world = world;
  for (auto &p : transformed_world) p.pnt = Q*p.pnt+offset;
  VoxelMap transformed_map(odom,local); transformed_map.cut_voxel(transformed_world,0,0);
  auto transformed = prior;
  transformed.R = Q*prior.R; transformed.p = Q*prior.p+offset;
  transformed.v = Q*prior.v; transformed.g = Q*prior.g;
  Eigen::Matrix<double,STATE_DOF,STATE_DOF> W = Eigen::Matrix<double,STATE_DOF,STATE_DOF>::Identity();
  W.block<3,3>(3,3)=Q; W.block<3,3>(6,6)=Q;
  transformed.cov = W*prior.cov*W.transpose();
  assert(Ricp(cfg).observe(transformed,source,transformed_map).accepted);
  assert((transformed.R-Q*state.R).norm()<1e-7 && (transformed.p-Q*state.p-offset).norm()<1e-7);
  assert((transformed.v-Q*state.v).norm()<1e-7 && (transformed.ba-state.ba).norm()<1e-7);
  assert((transformed.cov-W*state.cov*W.transpose()).norm()<1e-7);
  StateGroup distant = prior; distant.p.setConstant(100); const auto snapshot = distant;
  assert(!Ricp(cfg).observe(distant,source,map).accepted);
  assert(distant.p==snapshot.p && distant.v==snapshot.v && distant.R==snapshot.R && distant.bg==snapshot.bg && distant.ba==snapshot.ba && distant.g==snapshot.g && distant.t==snapshot.t);
  assert((distant.cov.array()==snapshot.cov.array()).all());
  auto invalid = source; invalid.back().pnt.x() = std::numeric_limits<double>::quiet_NaN();
  assert(!Ricp(cfg).observe(distant,invalid,map).accepted);
  assert((distant.cov.array()==snapshot.cov.array()).all() && distant.p==snapshot.p);
  std::cout << "PASS: library solve, nonidentity rotation/translation, nonplane moments, velocity/bias fusion, covariance and rollback\n";
}
