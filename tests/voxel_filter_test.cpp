#include <sapphire/odometry/voxel_filter.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

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

void expectNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message)
{
    if (std::abs(actual - expected) > tolerance) {
        fail(message);
    }
}

sapphire::Point makePoint(
    float x,
    float y,
    float z,
    float intensity,
    double timestamp)
{
    sapphire::Point point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.data[3] = 1.0f;
    point.intensity = intensity;
    point.timestamp = timestamp;
    return point;
}

void testCentroidsPreserveFields() {
    sapphire::PointCloud cloud;
    cloud.push_back(makePoint(0.1f, 0.2f, 0.3f, 2.0f, 1.0));
    cloud.push_back(makePoint(0.3f, 0.4f, 0.5f, 4.0f, 3.0));

    const auto result =
        sapphire::deterministicVoxelDownsample(cloud, 1.0);
    expect(result->size() == 1, "same-voxel points must be merged");
    expectNear(result->front().x, 0.2, 1e-6, "centroid X");
    expectNear(result->front().y, 0.3, 1e-6, "centroid Y");
    expectNear(result->front().z, 0.4, 1e-6, "centroid Z");
    expectNear(result->front().intensity, 3.0, 1e-6, "mean intensity");
    expectNear(result->front().timestamp, 2.0, 1e-12, "mean timestamp");
    expectNear(result->front().data[3], 1.0, 1e-6, "homogeneous W");
}

void testNegativeCoordinatesUseFloor() {
    sapphire::PointCloud cloud;
    cloud.push_back(makePoint(-0.1f, 0.0f, 0.0f, 1.0f, 0.0));
    cloud.push_back(makePoint(0.1f, 0.0f, 0.0f, 2.0f, 0.0));

    const auto result =
        sapphire::deterministicVoxelDownsample(cloud, 1.0);
    expect(result->size() == 2,
           "negative and positive points must occupy different voxels");
    expect(result->points[0].x < 0.0f,
           "voxel output must be ordered by signed coordinate");
}

void testParallelPathIsDeterministic() {
    sapphire::PointCloud cloud;
    constexpr size_t voxel_count = 100;
    constexpr size_t points_per_voxel = 50;
    cloud.reserve(voxel_count * points_per_voxel);
    for (size_t voxel = 0; voxel < voxel_count; ++voxel) {
        for (size_t sample = 0; sample < points_per_voxel; ++sample) {
            cloud.push_back(makePoint(
                static_cast<float>(voxel)
                    + 0.1f
                    + static_cast<float>(sample) * 0.01f,
                0.2f,
                0.3f,
                static_cast<float>(sample),
                static_cast<double>(sample) * 0.001));
        }
    }

    const auto first =
        sapphire::deterministicVoxelDownsample(cloud, 1.0, 4);
    const auto second =
        sapphire::deterministicVoxelDownsample(cloud, 1.0, 4);
    expect(first->size() == voxel_count,
           "parallel filtering must emit exactly one point per voxel");
    expect(second->size() == first->size(),
           "parallel filtering must preserve output count");
    for (size_t i = 0; i < first->size(); ++i) {
        const auto& lhs = first->points[i];
        const auto& rhs = second->points[i];
        expect(lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z,
               "parallel centroid coordinates must be repeatable");
        expect(lhs.intensity == rhs.intensity,
               "parallel intensity must be repeatable");
        expect(lhs.timestamp == rhs.timestamp,
               "parallel timestamp must be repeatable");
    }
}

void testInvalidLeafSizeRejected() {
    bool threw = false;
    try {
        sapphire::deterministicVoxelDownsample(
            sapphire::PointCloud{}, 0.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    expect(threw, "zero leaf size must be rejected");
}

}  // namespace

int main() {
    testCentroidsPreserveFields();
    testNegativeCoordinatesUseFloor();
    testParallelPathIsDeterministic();
    testInvalidLeafSizeRejected();
    std::cout << "All voxel filter tests passed\n";
    return EXIT_SUCCESS;
}
