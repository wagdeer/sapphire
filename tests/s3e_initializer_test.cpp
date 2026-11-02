#include "module/initialize.hpp"
#include <cassert>
#include <iostream>
using namespace sapphire;
int main() {
  for (const Eigen::Vector3d g : {Eigen::Vector3d(0,0,-9.81), Eigen::Vector3d(0,0,9.81), Eigen::Vector3d(.4,-.3,-9.79), Eigen::Vector3d(1e-12,-1e-12,-9.81)}) {
    std::vector<StateGroup> xs(2); xs[0].g=xs[1].g=g; xs[1].p=Eigen::Vector3d(1,2,3);
    const double distance=(xs[1].p-xs[0].p).norm();
    Initialization::instance().align_gravity(xs);
    for (const auto &x:xs) { assert(x.R.allFinite()); assert((x.R.transpose()*x.R-Eigen::Matrix3d::Identity()).norm()<1e-12); assert(x.g.allFinite()); assert(x.g.head<2>().norm()<1e-10); }
    assert(std::abs((xs[1].p-xs[0].p).norm()-distance)<1e-10);
  }
  LocalSubmapParameters cfg; cfg.thread_num=1;
  tools::ParallelExecutor<> executor(1); LI_BA_OptimizerGravity optimizer(cfg,executor);
  Eigen::MatrixXd H; std::vector<double> costs;
  std::vector<StateGroup> empty_states(1); LidarFactor empty(1); std::deque<std::unique_ptr<ImuFactor>> no_imu;
  optimizer.damping_iter(empty_states,empty,no_imu,costs,&H,1);
  assert(costs.size()==2 && costs[0]==0 && costs[1]==0 && empty_states[0].R.allFinite());

  std::deque<ImuMeas> samples;
  for(int i=0;i<=10;++i) { ImuMeas m; m.timestamp=.01*i; m.gyro.setZero(); m.accel=Eigen::Vector3d(0,0,9.81); samples.push_back(m); }
  for (double disturbance : {0., .01, .3, 1.}) {
    std::vector<StateGroup> states(2); for(auto &x:states)x.g=Eigen::Vector3d(0,0,-9.81);
    states[1].p=Eigen::Vector3d(disturbance,-.3*disturbance,.2*disturbance);
    states[1].R=lie::SO3d::exp(Eigen::Vector3d(.1,-.2,.3)*disturbance).R();
    std::deque<std::unique_ptr<ImuFactor>> factors;
    factors.push_back(std::make_unique<ImuFactor>()); factors.back()->push_imu(samples,1,cfg);
    LidarFactor lidar(2);
    costs.clear();
    optimizer.damping_iter(states,lidar,factors,costs,&H,3);
    Eigen::MatrixXd h(33,33); Eigen::VectorXd j(33);
    const double actual=cfg.imu_coef*.5*factors.front()->give_evaluate_g(states[0],states[1],h,j,false);
    assert(costs.size()==2);assert(std::isfinite(costs[1]));
    assert(std::abs(actual-costs[1])<1e-8*std::max(1.,std::abs(actual)));
    assert(costs[1]<=costs[0]+1e-10);
  }
  std::cout<<"Gravity alignment, no-information rejection, stationary and accepted-state costs passed\n";
}
