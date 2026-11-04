#include <sapphire/io/tum.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

[[noreturn]] void fail(const std::string& message) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void expect(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

std::vector<std::string> readLines(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        fail("unable to read " + path.string());
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }
    return lines;
}

std::vector<double> fieldsOf(const std::string& line) {
    std::istringstream stream(line);
    stream.imbue(std::locale::classic());
    std::vector<double> values;
    double value = 0.0;
    while (stream >> value) {
        values.push_back(value);
    }
    return values;
}

sapphire::MapTrajectorySample sampleAt(
    double stamp,
    const Eigen::Vector3d& translation,
    const Eigen::Quaterniond& rotation)
{
    sapphire::MapTrajectorySample sample;
    sample.stamp = stamp;
    sample.T_map_lidar = sapphire::Isometry3d::Identity();
    sample.T_map_lidar.translation() = translation;
    sample.T_map_lidar.linear() = rotation.toRotationMatrix();
    return sample;
}

void expectQuaternion(
    const std::vector<double>& fields,
    const Eigen::Quaterniond& rotation,
    const std::string& message)
{
    expect(fields.size() == 8, message + " must have 8 TUM fields");
    const Eigen::Quaterniond stored(rotation.toRotationMatrix());
    expect(std::abs(fields[4] - stored.x()) < 5e-10, message + " qx");
    expect(std::abs(fields[5] - stored.y()) < 5e-10, message + " qy");
    expect(std::abs(fields[6] - stored.z()) < 5e-10, message + " qz");
    expect(std::abs(fields[7] - stored.w()) < 5e-10, message + " qw");
    const bool wxyz = std::abs(fields[4] - stored.w()) < 5e-10
        && std::abs(fields[5] - stored.x()) < 5e-10
        && std::abs(fields[6] - stored.y()) < 5e-10
        && std::abs(fields[7] - stored.z()) < 5e-10;
    const bool same_as_xyzw = std::abs(stored.w() - stored.x()) < 5e-10
        && std::abs(stored.x() - stored.y()) < 5e-10
        && std::abs(stored.y() - stored.z()) < 5e-10;
    expect(!wxyz || same_as_xyzw, message + " must stay xyzw, not wxyz");
}

void testStandardQuaternionOrder() {
    const auto directory =
        std::filesystem::temp_directory_path() / "sapphire_tum_order";
    std::filesystem::remove_all(directory);
    const auto path = directory / "sapphire" / "sequence" / "alpha.tum";

    const Eigen::Quaterniond rotation(
        std::sqrt(16.0 / 30.0),
        std::sqrt(1.0 / 30.0),
        -std::sqrt(4.0 / 30.0),
        std::sqrt(9.0 / 30.0));
    const Eigen::Quaterniond identity(1.0, 0.0, 0.0, 0.0);
    sapphire::MapTrajectory trajectory;
    trajectory.samples.push_back(
        sampleAt(10.0, Eigen::Vector3d(1.25, -2.5, 0.5), rotation));
    trajectory.samples.push_back(
        sampleAt(10.1, Eigen::Vector3d(1.5, -2.5, 0.5), identity));
    trajectory.samples.push_back(
        sampleAt(10.1, Eigen::Vector3d(9.0, 9.0, 9.0), identity));

    const sapphire::TumWriteResult result =
        sapphire::writeTumTrajectory(path, trajectory);
    expect(result.written == 2, "duplicate timestamp must be skipped");
    expect(result.skipped == 1, "one non-increasing timestamp is skipped");

    const auto lines = readLines(path);
    expect(lines.size() == 2, "file keeps both strictly increasing poses");
    const auto first = fieldsOf(lines[0]);
    const auto second = fieldsOf(lines[1]);
    expect(std::abs(first[0] - 10.0) < 5e-10, "timestamp");
    expect(std::abs(first[1] - 1.25) < 5e-10, "tx");
    expect(std::abs(first[2] - (-2.5)) < 5e-10, "ty");
    expect(std::abs(first[3] - 0.5) < 5e-10, "tz");
    expectQuaternion(first, rotation, "oriented pose");
    expectQuaternion(second, identity, "identity pose");
    expect(
        std::abs(second[4]) < 5e-10 && std::abs(second[5]) < 5e-10
            && std::abs(second[6]) < 5e-10 && std::abs(second[7] - 1.0) < 5e-10,
        "identity orientation stays 0 0 0 1");
    std::filesystem::remove_all(directory);
}

void testEmptyTrajectoryCreatesNoFile() {
    const auto path =
        std::filesystem::temp_directory_path() / "sapphire_tum_empty.tum";
    std::filesystem::remove(path);
    const sapphire::TumWriteResult result =
        sapphire::writeTumTrajectory(path, {});
    expect(result.written == 0, "empty trajectory writes nothing");
    expect(!std::filesystem::exists(path), "empty trajectory creates no file");
}

void testSaveStatusKeepsEmptyAndUnconfiguredSuccessful() {
    const auto unconfigured = sapphire::saveMapTrajectory({}, {});
    expect(
        unconfigured.status == sapphire::TrajectorySaveStatus::NotConfigured,
        "an empty output path is not configured");
    expect(
        sapphire::trajectorySaveSucceeded(unconfigured.status),
        "an unconfigured trajectory output is successful");
    expect(unconfigured.error.empty(), "an unconfigured save has no error");

    const auto path =
        std::filesystem::temp_directory_path() / "sapphire_tum_save_empty.tum";
    std::filesystem::remove(path);
    const auto empty = sapphire::saveMapTrajectory(path, {});
    expect(
        empty.status == sapphire::TrajectorySaveStatus::Empty,
        "a configured empty trajectory is empty, not an I/O failure");
    expect(
        sapphire::trajectorySaveSucceeded(empty.status),
        "an empty trajectory does not fail the process");
    expect(!std::filesystem::exists(path), "an empty save creates no file");
    expect(empty.error.empty(), "an empty save has no error");
}

void testSaveStatusReportsIoFailure() {
    const auto blocked =
        std::filesystem::temp_directory_path() / "sapphire_tum_not_a_directory";
    std::filesystem::remove_all(blocked);
    {
        std::ofstream stream(blocked);
        expect(static_cast<bool>(stream), "unable to create blocking file");
        stream << "not a directory\n";
    }
    const auto path = blocked / "trajectory.tum";

    sapphire::MapTrajectory trajectory;
    trajectory.samples.push_back(sampleAt(
        1.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));
    const auto report = sapphire::saveMapTrajectory(path, trajectory);
    expect(
        report.status == sapphire::TrajectorySaveStatus::IoFailure,
        "a trajectory path under a file is an I/O failure");
    expect(
        !sapphire::trajectorySaveSucceeded(report.status),
        "an I/O failure must fail the process");
    expect(!report.error.empty(), "an I/O failure keeps the error text");
    std::filesystem::remove_all(blocked);
}

}  // namespace

int main() {
    testStandardQuaternionOrder();
    testEmptyTrajectoryCreatesNoFile();
    testSaveStatusKeepsEmptyAndUnconfiguredSuccessful();
    testSaveStatusReportsIoFailure();
    std::cout << "All TUM tests passed\n";
    return EXIT_SUCCESS;
}
