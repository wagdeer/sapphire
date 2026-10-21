// Copyright (c) 2021, Viktor Larsson
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of the copyright holder nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef POSELIB_ABSOLUTE_H_
#define POSELIB_ABSOLUTE_H_

#include "../../camera_pose.h"
#include "../../types.h"
#include "optim_utils.h"
#include "refiner_base.h"

namespace poselib {

template <typename ResidualWeightVector = UniformWeightVector, typename Accumulator = NormalAccumulator>
class PinholeAbsolutePoseRefiner : public RefinerBase<CameraPose, Accumulator> {
  public:
    PinholeAbsolutePoseRefiner(const std::vector<Point2D> &points2D, const std::vector<Point3D> &points3D,
                               const ResidualWeightVector &w = ResidualWeightVector())
        : x(points2D), X(points3D), weights(w) {
        this->num_params = 6;
    }

    double compute_residual(Accumulator &acc, const CameraPose &pose) {
        const Eigen::Matrix3d R = pose.R();
        for (std::size_t i = 0; i < x.size(); ++i) {
            const Eigen::Vector3d Z = R * X[i] + pose.t;
            if (Z(2) < 0)
                continue;
            const Eigen::Vector2d res = Z.hnormalized() - x[i];
            acc.add_residual(res, weights[i]);
        }
        return acc.get_residual();
    }

    void compute_jacobian(Accumulator &acc, const CameraPose &pose) {
        const Eigen::Matrix3d R = pose.R();
        Eigen::Matrix<double, 2, 3> Jproj;
        Eigen::Matrix<double, 2, 6> J;

        for (std::size_t i = 0; i < x.size(); ++i) {
            const Eigen::Vector3d Xi = X[i];
            const Eigen::Vector3d Z = R * Xi + pose.t;
            if (Z(2) < 0)
                continue;

            const Eigen::Vector2d zp = Z.hnormalized();
            Jproj << 1.0 / Z(2), 0, -zp(0) / Z(2), 0, 1.0 / Z(2), -zp(1) / Z(2);
            const Eigen::Vector2d res = zp - x[i];
            const Eigen::Matrix<double, 2, 3> dZ = Jproj * R;
            J.col(0) = -Xi(2) * dZ.col(1) + Xi(1) * dZ.col(2);
            J.col(1) = Xi(2) * dZ.col(0) - Xi(0) * dZ.col(2);
            J.col(2) = -Xi(1) * dZ.col(0) + Xi(0) * dZ.col(1);
            J.template block<2, 3>(0, 3) = dZ;
            acc.add_jacobian(res, J, weights[i]);
        }
    }

    CameraPose step(const Eigen::VectorXd &dp, const CameraPose &pose) const {
        CameraPose pose_new;
        pose_new.q = quat_step_post(pose.q, dp.block<3, 1>(0, 0));
        pose_new.t = pose.t + pose.rotate(dp.block<3, 1>(3, 0));
        return pose_new;
    }

    const std::vector<Point2D> &x;
    const std::vector<Point3D> &X;
    const ResidualWeightVector &weights;
    typedef CameraPose param_t;
};

} // namespace poselib

#endif
