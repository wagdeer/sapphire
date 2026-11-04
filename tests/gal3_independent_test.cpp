#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>

#include <Gal3.hpp>
#include <SO3.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

using Vec3 = Eigen::Vector3d;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat5 = Eigen::Matrix<double, 5, 5>;
using Mat10 = Eigen::Matrix<double, 10, 10>;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Gal3 = lie::Gal3<double>;
using SO3 = lie::SO3<double>;
using Integrator = sapphire::detail::MeanOnlyGal3Integrator;

const Vec3 kGravity(0.0, 0.0, -9.80665);

void expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expectNear(double actual, double expected, double tolerance,
                const char* message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message);
    }
}

Mat3 so3Exp(const Vec3& omega) {
    return SO3::exp(omega).R();
}

Vec3 so3Log(const Mat3& rotation) {
    return SO3::log(SO3(rotation));
}

Mat3 projectToSo3(const Mat3& matrix) {
    Eigen::JacobiSVD<Mat3> svd(
        matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 rotation = svd.matrixU() * svd.matrixV().transpose();
    if (rotation.determinant() < 0.0) {
        rotation = -rotation;
    }
    return rotation;
}

Mat5 taylorExp(const Vec10& u) {
    Mat5 x = Mat5::Zero();
    x.block<3, 3>(0, 0) = SO3::wedge(u.head<3>());
    x.block<3, 1>(0, 3) = u.segment<3>(3);
    x.block<3, 1>(0, 4) = u.segment<3>(6);
    x(3, 4) = u(9);

    Mat5 term = Mat5::Identity();
    Mat5 sum = Mat5::Zero();
    for (int k = 0; k < 40; ++k) {
        sum += term;
        term = (term * x) / static_cast<double>(k + 1);
    }
    return sum;
}

struct NavState {
    Mat3 rotation = Mat3::Identity();
    Vec3 velocity = Vec3::Zero();
    Vec3 position = Vec3::Zero();
};

NavState rk4Step(
    const NavState& state,
    const Vec3& omega,
    const Vec3& accel,
    double dt,
    int substeps)
{
    NavState value = state;
    const Mat3 omega_hat = SO3::wedge(omega);
    const double h = dt / static_cast<double>(substeps);
    for (int i = 0; i < substeps; ++i) {
        const Mat3 k1_r = value.rotation * omega_hat;
        const Vec3 k1_v = value.rotation * accel + kGravity;
        const Vec3 k1_p = value.velocity;

        const Mat3 r2 = value.rotation + 0.5 * h * k1_r;
        const Mat3 k2_r = r2 * omega_hat;
        const Vec3 k2_v = r2 * accel + kGravity;
        const Vec3 k2_p = value.velocity + 0.5 * h * k1_v;

        const Mat3 r3 = value.rotation + 0.5 * h * k2_r;
        const Mat3 k3_r = r3 * omega_hat;
        const Vec3 k3_v = r3 * accel + kGravity;
        const Vec3 k3_p = value.velocity + 0.5 * h * k2_v;

        const Mat3 r4 = value.rotation + h * k3_r;
        const Mat3 k4_r = r4 * omega_hat;
        const Vec3 k4_v = r4 * accel + kGravity;
        const Vec3 k4_p = value.velocity + h * k3_v;

        value.rotation +=
            (h / 6.0)
            * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r);
        value.velocity +=
            (h / 6.0)
            * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
        value.position +=
            (h / 6.0)
            * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p);
    }
    value.rotation = projectToSo3(value.rotation);
    return value;
}

Gal3 makeUpsilon(const Vec3& omega, const Vec3& accel, double dt) {
    Vec10 input;
    input << omega, accel, Vec3::Zero(), 1.0;
    return Gal3::exp(input * dt);
}

NavState recover(const Gal3& upsilon, const NavState& initial) {
    const Gal3 x0(
        initial.rotation,
        Gal3::IsometriesType{initial.velocity, initial.position},
        0.0);
    const Gal3 result = sapphire::detail::recoverWorldState(
        upsilon, x0, kGravity);
    return {result.R(), result.v(), result.p()};
}

double navDifference(const NavState& lhs, const NavState& rhs) {
    return std::max({
        so3Log(lhs.rotation.transpose() * rhs.rotation).norm(),
        (lhs.velocity - rhs.velocity).norm(),
        (lhs.position - rhs.position).norm(),
    });
}

void testGroupIdentities() {
    std::mt19937 rng(42);
    std::normal_distribution<double> normal(0.0, 1.0);
    double max_round_trip = 0.0;
    double max_adjoint = 0.0;
    double max_right_jacobian = 0.0;
    double max_inverse = 0.0;

    for (int trial = 0; trial < 200; ++trial) {
        Vec10 u;
        u << 0.3 * normal(rng), 0.3 * normal(rng), 0.3 * normal(rng),
            normal(rng), normal(rng), normal(rng),
            normal(rng), normal(rng), normal(rng), 0.5 * normal(rng);
        const Gal3 x = Gal3::exp(u);
        max_round_trip = std::max(
            max_round_trip, (Gal3::log(x) - u).norm());

        const Gal3 chi(
            so3Exp(Vec3(normal(rng), normal(rng), normal(rng))),
            Gal3::IsometriesType{
                Vec3(normal(rng), normal(rng), normal(rng)),
                Vec3(normal(rng), normal(rng), normal(rng))},
            normal(rng));
        const Gal3 conjugated = chi * x * chi.inv();
        max_adjoint = std::max(
            max_adjoint,
            (conjugated.asMatrix()
             - Gal3::exp(chi.Adjoint() * u).asMatrix())
                .cwiseAbs()
                .maxCoeff());

        Mat10 numeric;
        constexpr double epsilon = 1e-7;
        for (int column = 0; column < 10; ++column) {
            Vec10 delta = Vec10::Zero();
            delta(column) = 1.0;
            numeric.col(column) =
                Gal3::log(x.inv() * Gal3::exp(u + epsilon * delta))
                / epsilon;
        }
        max_right_jacobian = std::max(
            max_right_jacobian,
            (numeric - Gal3::rightJacobian(u)).cwiseAbs().maxCoeff());
        max_inverse = std::max(
            max_inverse,
            (Gal3::invLeftJacobian(u) * Gal3::leftJacobian(u)
             - Mat10::Identity())
                .cwiseAbs()
                .maxCoeff());
    }

    expect(max_round_trip < 1e-9, "Gal3 exp/log round trip");
    expect(max_adjoint < 1e-9, "Gal3 adjoint conjugation");
    expect(max_right_jacobian < 1e-6, "Gal3 right Jacobian");
    expect(max_inverse < 1e-9, "Gal3 inverse left Jacobian");
}

void testMeanAgainstRk4() {
    NavState initial;
    initial.rotation = so3Exp(Vec3(0.3, -0.2, 1.1));
    initial.velocity = Vec3(0.4, -1.2, 0.7);
    initial.position = Vec3(2.0, -3.0, 0.5);
    const Vec3 omega(0.4, -0.9, 1.7);
    const Vec3 accel(0.8, -0.2, 9.9);

    for (const double dt : {0.005, 0.02, 0.1}) {
        const NavState recovered =
            recover(makeUpsilon(omega, accel, dt), initial);
        const NavState reference =
            rk4Step(initial, omega, accel, dt, 200000);
        expect(
            navDifference(recovered, reference) < 1e-9,
            "Gal3 mean propagation must match RK4");
    }

    Integrator integrator;
    NavState reference = initial;
    for (int i = 0; i < 400; ++i) {
        const double t = static_cast<double>(i) * 0.005;
        const Vec3 wi(
            0.4 * std::sin(0.7 * t),
            -0.9 * std::cos(0.4 * t),
            1.7);
        const Vec3 ai(
            0.8 * std::cos(1.1 * t),
            -0.2,
            9.9 + 0.3 * std::sin(0.3 * t));
        integrator.integrate(ai, wi, 0.005);
        reference = rk4Step(reference, wi, ai, 0.005, 2000);
    }
    expect(
        navDifference(recover(integrator.Upsilon(), initial), reference)
            < 1e-9,
        "chained Gal3 propagation must match chained RK4");
}

void testRecoveryRequiresZeroTimeComponent() {
    const Gal3 x0(
        Mat3::Identity(),
        Gal3::IsometriesType{Vec3(1.0, 0.0, 0.0), Vec3::Zero()},
        3.0);
    bool threw = false;
    try {
        (void)sapphire::detail::recoverWorldState(
            makeUpsilon(Vec3(0.1, 0.2, 0.3), Vec3::Zero(), 0.5),
            x0,
            kGravity);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(
        threw,
        "recoverWorldState must reject a non-zero initial time component");
}

Vec15 errorState(const NavState& truth, const NavState& nominal) {
    Vec15 error = Vec15::Zero();
    error.segment<3>(0) =
        so3Log(nominal.rotation.transpose() * truth.rotation);
    error.segment<3>(3) = truth.velocity - nominal.velocity;
    error.segment<3>(6) = truth.position - nominal.position;
    return error;
}

NavState propagateWithBiasError(
    const NavState& state,
    const Vec3& omega,
    const Vec3& accel,
    const Vec3& accel_bias_error,
    const Vec3& gyro_bias_error,
    double dt)
{
    return recover(
        makeUpsilon(
            omega - gyro_bias_error,
            accel - accel_bias_error,
            dt),
        state);
}

void testEskfLinearization() {
    for (const double dt : {0.001, 0.005, 0.02}) {
        NavState initial;
        initial.rotation = so3Exp(Vec3(0.25, -0.4, 0.9));
        initial.velocity = Vec3(0.5, -0.8, 0.3);
        initial.position = Vec3(1.0, 2.0, -0.5);
        const Vec3 omega(0.35, -0.7, 1.4);
        const Vec3 accel(0.6, -0.15, 9.85);

        Mat15 f = Mat15::Zero();
        f.block<3, 3>(0, 0) = -SO3::wedge(omega);
        f.block<3, 3>(0, 12) = -Mat3::Identity();
        f.block<3, 3>(3, 0) = -initial.rotation * SO3::wedge(accel);
        f.block<3, 3>(3, 9) = -initial.rotation;
        f.block<3, 3>(6, 3) = Mat3::Identity();
        const Mat15 model = Mat15::Identity() + f * dt;

        const NavState nominal = propagateWithBiasError(
            initial, omega, accel, Vec3::Zero(), Vec3::Zero(), dt);
        Mat15 numeric;
        constexpr double epsilon = 1e-6;
        for (int column = 0; column < 15; ++column) {
            Vec15 delta = Vec15::Zero();
            delta(column) = 1.0;
            const auto evaluate = [&](double sign) {
                const Vec15 offset = sign * epsilon * delta;
                NavState state = initial;
                state.rotation *= so3Exp(offset.segment<3>(0));
                state.velocity += offset.segment<3>(3);
                state.position += offset.segment<3>(6);
                return errorState(
                    propagateWithBiasError(
                        state,
                        omega,
                        accel,
                        offset.segment<3>(9),
                        offset.segment<3>(12),
                        dt),
                    nominal);
            };
            numeric.col(column) =
                (evaluate(1.0) - evaluate(-1.0)) / (2.0 * epsilon);
        }

        double max_error = 0.0;
        for (int row = 0; row < 9; ++row) {
            for (int column = 0; column < 15; ++column) {
                max_error = std::max(
                    max_error,
                    std::abs(numeric(row, column) - model(row, column)));
            }
        }
        expect(
            max_error < 9.85 * dt * dt + 1e-6,
            "ESKF first-order linearization must differ only by O(dt^2)");
    }
}

void testGoldenVectorAndOrdering() {
    Vec10 xi;
    xi << 0.3267045270397072, -0.21318895514136532,
        -0.1810111529904679, -0.24823309993679712,
        -0.3282613511681929, -0.3614305580886979,
        -0.6659680127970163, 0.0801034296770802,
        -0.005425197747099379, -0.11137002094819903;

    Mat5 expected;
    expected <<
        0.961491754653074, 0.14119138670272793, -0.23579354964696070,
        -0.22904001980446076, -0.63732167303197800,
        -0.20977429976081086, 0.93131798263194740,
        -0.29772732220308690, -0.23988916442951638,
        0.16116104619552252, 0.17756223949368968,
        0.33572579219851434, 0.92507288553857480,
        -0.43087107476209770, -0.03248254605908948,
        0.0, 0.0, 0.0, 1.0, -0.11137002094819903,
        0.0, 0.0, 0.0, 0.0, 1.0;

    expect(
        (Gal3::exp(xi).asMatrix() - expected).cwiseAbs().maxCoeff()
            < 1e-12,
        "Gal3 exp must match the GTSAM golden vector");
    expect(
        (Gal3::exp(xi).asMatrix() - taylorExp(xi))
                .cwiseAbs()
                .maxCoeff()
            < 1e-14,
        "Gal3 exp must match an independent matrix Taylor series");

    Vec10 probe = Vec10::Zero();
    probe(3) = 1.0;
    expect(Gal3::wedge(probe)(0, 3) == 1.0, "velocity tangent ordering");
    probe.setZero();
    probe(6) = 1.0;
    expect(Gal3::wedge(probe)(0, 4) == 1.0, "position tangent ordering");
    probe.setZero();
    probe(9) = 1.0;
    expect(Gal3::wedge(probe)(3, 4) == 1.0, "time tangent ordering");
}

void testSmallAngleStability() {
    const Vec3 accel(0.8, -0.2, 9.9);
    const Vec3 direction = Vec3(0.3, -0.5, 0.81).normalized();
    double worst_error = 0.0;
    double worst_magnitude = 0.0;
    for (const double magnitude :
         {1e-2, 1e-4, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10}) {
        Vec10 u;
        u << magnitude * direction, accel * 0.005, Vec3::Zero(), 0.005;
        const double error =
            (Gal3::exp(u).asMatrix() - taylorExp(u))
                .cwiseAbs()
                .maxCoeff();
        if (error > worst_error) {
            worst_error = error;
            worst_magnitude = magnitude;
        }
    }
    if (worst_error >= 5e-10) {
        std::ostringstream message;
        message << "Gal3 exp small-angle error " << worst_error
                << " at magnitude " << worst_magnitude;
        throw std::runtime_error(message.str());
    }
}

}  // namespace

int main() {
    try {
        testGroupIdentities();
        testMeanAgainstRk4();
        testRecoveryRequiresZeroTimeComponent();
        testEskfLinearization();
        testGoldenVectorAndOrdering();
        testSmallAngleStability();
        std::cout << "All independent Gal(3) tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Independent Gal(3) test failed: " << error.what()
                  << '\n';
        return EXIT_FAILURE;
    }
}
