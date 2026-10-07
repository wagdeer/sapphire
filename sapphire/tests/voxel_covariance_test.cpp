#include "frontend/eskf/voxel_map.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace sapphire;
int main() {
  OdometryParameters odom; LocalSubmapParameters local; OctoTree::configure(odom, local);
  std::mt19937 rng(27); std::normal_distribution<double> noise(0, 1);
  for (double offset : {0., 100., 1000., 10000.}) {
    OctoTree tree(local.max_layer, local.win_size);
    Eigen::Matrix<double,9,9> reference = Eigen::Matrix<double,9,9>::Zero();
    for (int i=0;i<1000;++i) {
      pointVar p; p.pnt = Eigen::Vector3d(offset + .3*noise(rng), -.7*offset + .3*noise(rng), .4*offset + .001*noise(rng));
      Eigen::Matrix3d a; for (int j=0;j<9;++j) a.data()[j]=noise(rng);
      p.var = a*a.transpose()*1e-4;
      const double x=p.pnt.x(), y=p.pnt.y(), z=p.pnt.z();
      Eigen::Matrix<double,9,3> J;
      J << 2*x,0,0, y,x,0, z,0,x, 0,2*y,0, 0,z,y, 0,0,2*z, 1,0,0, 0,1,0, 0,0,1;
      reference += J*p.var*J.transpose();
      tree.push_fix(p);
    }
    const auto expanded=tree.moment_covariance();
    assert(expanded.isApprox(reference, 1e-13));
    assert(expanded.isApprox(expanded.transpose(), 0.));
    assert(sizeof(tree.cov_add)==45*sizeof(double));
    // Independent uncertainty projection, not merely a pack/unpack round trip.
    for (int k=0;k<10;++k) {
      Eigen::Matrix<double,9,1> q; for (int j=0;j<9;++j) q[j]=noise(rng);
      const double expected=q.dot(reference*q), actual=q.dot(expanded*q);
      assert(std::abs(actual-expected)<=1e-12*std::max(1.,std::abs(expected)));
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(tree.pcr_add.cov());
    tree.eig_value=solver.eigenvalues(); tree.eig_vector=solver.eigenvectors();tree.plane_update();
    assert(tree.plane.plane_var.allFinite());
    assert(std::abs(tree.plane.normal.norm()-1.)<1e-12);
    assert(std::abs(tree.plane.normal.z())>.999);
    // Expansion does not alter PSD apart from floating-point cancellation.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> uncertainty(tree.plane.plane_var);
    assert(uncertainty.eigenvalues().minCoeff()>-1e-8);
  }
  std::cout << "packed moment covariance PASS node_bytes=" << sizeof(OctoTree) << '\n';
}
