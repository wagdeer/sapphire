#include <sapphire/mapping/occupancy_grid.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

sapphire::PointCloudPtr makeWallCloud() {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    // Ground plane points (should free-ray only).
    for (int x = 0; x < 20; ++x) {
        sapphire::Point ground{};
        ground.x = 0.5f + 0.2f * static_cast<float>(x);
        ground.y = 0.0f;
        ground.z = -0.4f;
        ground.data[3] = 1.0f;
        cloud->push_back(ground);
    }
    // Vertical wall at x≈4m, z in obstacle band.
    for (int z = 0; z < 8; ++z) {
        for (int y = -3; y <= 3; ++y) {
            sapphire::Point wall{};
            wall.x = 4.0f;
            wall.y = 0.15f * static_cast<float>(y);
            wall.z = 0.1f + 0.15f * static_cast<float>(z);
            wall.data[3] = 1.0f;
            cloud->push_back(wall);
        }
    }
    return cloud;
}

int8_t sampleCell(
    const sapphire::OccupancyGridMsg& msg,
    double x,
    double y)
{
    // Match worldToGlobalIndex: point coords are stored as float
    // (Point::x/y/z) and promoted to double before arithmetic.
    // Float-only division (fx/freso) can undershoot due to reciprocal
    // precision; keep the arithmetic in double with float-rounded inputs.
    const double dx = static_cast<double>(static_cast<float>(x));
    const double dy = static_cast<double>(static_cast<float>(y));
    const int gx = static_cast<int>(
        std::floor((dx - msg.origin_x) / msg.resolution));
    const int gy = static_cast<int>(
        std::floor((dy - msg.origin_y) / msg.resolution));
    expect(gx >= 0 && gy >= 0 && gx < msg.width && gy < msg.height,
        "sample must land inside the exported grid");
    return msg.data[static_cast<size_t>(gy * msg.width + gx)];
}

void testAttitudePlaneMarksWallOccupied() {
    sapphire::Config::Pgo::Occupancy options;
    options.enabled = true;
    options.resolution = 0.1;
    options.h_clearance = 0.15;
    options.d_max = 1.5;
    options.occ_threshold = 0.3;
    options.usable_range = 20.0;
    options.min_range = 0.2;
    options.cloud_voxel_size = 0.0;
    options.margin = 2.0;

    sapphire::OccupancyGrid grid(options);
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
    grid.insertScan(makeWallCloud(), pose);

    // Reinforced with a second overlapping scan so visit/hit converge.
    grid.insertScan(makeWallCloud(), pose);
    grid.insertScan(makeWallCloud(), pose);
    grid.insertScan(makeWallCloud(), pose);

    const sapphire::OccupancyGridMsg msg = grid.toMsg();
    expect(msg.width > 0 && msg.height > 0, "grid must be non-empty");
    expect(sampleCell(msg, 4.0, 0.0) == 100, "wall cell must be occupied");
    expect(sampleCell(msg, 1.5, 0.0) == 0, "space before wall must be free");
}

void testGroundFreeRaysDoNotEraseWall() {
    sapphire::Config::Pgo::Occupancy options;
    options.enabled = true;
    options.resolution = 0.1;
    options.h_clearance = 0.15;
    options.d_max = 1.5;
    options.occ_threshold = 0.3;
    options.usable_range = 20.0;
    options.min_range = 0.2;
    options.cloud_voxel_size = 0.0;
    options.margin = 2.0;

    auto cloud = makeWallCloud();
    // Dense ground returns through/near the wall (out of obstacle band).
    for (int i = 0; i < 200; ++i) {
        sapphire::Point ground{};
        ground.x = 0.3f + 0.02f * static_cast<float>(i);
        ground.y = 0.0f;
        ground.z = -0.8f;
        ground.data[3] = 1.0f;
        cloud->push_back(ground);
    }

    sapphire::OccupancyGrid grid(options);
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
    for (int i = 0; i < 4; ++i) {
        grid.insertScan(cloud, pose);
    }

    const sapphire::OccupancyGridMsg msg = grid.toMsg();
    expect(sampleCell(msg, 4.0, 0.0) == 100,
        "dense out-of-band ground rays must not erase the wall");
}

void testCeilingFilteredByDMax() {
    sapphire::Config::Pgo::Occupancy options;
    options.enabled = true;
    options.resolution = 0.1;
    options.h_clearance = 0.1;
    options.d_max = 1.0;
    options.occ_threshold = 0.3;
    options.usable_range = 20.0;
    options.min_range = 0.2;
    options.cloud_voxel_size = 0.0;
    options.margin = 2.0;

    auto cloud = std::make_shared<sapphire::PointCloud>();
    for (int i = 0; i < 20; ++i) {
        sapphire::Point ceiling{};
        ceiling.x = 3.0f;
        ceiling.y = 0.1f * static_cast<float>(i - 10);
        ceiling.z = 2.5f;  // above d_max
        ceiling.data[3] = 1.0f;
        cloud->push_back(ceiling);
    }

    sapphire::OccupancyGrid grid(options);
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
    for (int i = 0; i < 4; ++i) {
        grid.insertScan(cloud, pose);
    }

    const sapphire::OccupancyGridMsg msg = grid.toMsg();
    // Cropped export may be empty when every return is out of band.
    if (msg.width > 0 && msg.height > 0) {
        expect(sampleCell(msg, 3.0, 0.0) != 100,
            "ceiling above d_max must not become occupied");
    }
}

void testGroundMarginSkipsNearGroundHits() {
    // Mimic Mid-360 roof mount: h_clearance ≈ sensor height, ground at d≈-2.
    // Without ground_margin those returns are in-band HITs and paint the floor.
    sapphire::Config::Pgo::Occupancy options;
    options.enabled = true;
    options.resolution = 0.1;
    options.h_clearance = 2.0;
    options.ground_margin = 0.3;
    options.d_max = 6.0;
    options.occ_threshold = 0.3;
    options.usable_range = 20.0;
    options.min_range = 0.2;
    options.cloud_voxel_size = 0.0;
    options.margin = 2.0;

    auto cloud = std::make_shared<sapphire::PointCloud>();
    // Near-ground returns (should be out of HIT band: d < -1.7).
    for (int i = 0; i < 40; ++i) {
        sapphire::Point ground{};
        ground.x = 1.0f + 0.05f * static_cast<float>(i);
        ground.y = 0.0f;
        ground.z = -1.95f;
        ground.data[3] = 1.0f;
        cloud->push_back(ground);
    }
    // Wall in the kept band (d = -0.5 >= -1.7).
    for (int y = -3; y <= 3; ++y) {
        for (int k = 0; k < 4; ++k) {
            sapphire::Point wall{};
            wall.x = 3.0f;
            wall.y = 0.1f * static_cast<float>(y);
            wall.z = -0.5f + 0.1f * static_cast<float>(k);
            wall.data[3] = 1.0f;
            cloud->push_back(wall);
        }
    }

    sapphire::OccupancyGrid grid(options);
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();
    for (int i = 0; i < 4; ++i) {
        grid.insertScan(cloud, pose);
    }

    const sapphire::OccupancyGridMsg msg = grid.toMsg();
    expect(msg.width > 0 && msg.height > 0, "grid must be non-empty");
    expect(sampleCell(msg, 3.0, 0.0) == 100,
        "in-band wall must still become occupied");
    expect(sampleCell(msg, 1.5, 0.0) != 100,
        "near-ground returns must not paint occupied cells");
}

sapphire::PointCloudPtr makeWallAtX(float x) {
    auto cloud = std::make_shared<sapphire::PointCloud>();
    for (int z = 0; z < 8; ++z) {
        for (int y = -3; y <= 3; ++y) {
            sapphire::Point wall{};
            wall.x = x;
            wall.y = 0.15f * static_cast<float>(y);
            wall.z = 0.1f + 0.15f * static_cast<float>(z);
            wall.data[3] = 1.0f;
            cloud->push_back(wall);
        }
    }
    return cloud;
}

void testDepthJitterClearsPreviousWallCell() {
    // Pose / ranging jitter moves the wall endpoint one cell farther. The old
    // cell lies on the new ray as a same-height miss; without clear_height_eps
    // it stays permanently occupied → multi-layer walls.
    sapphire::Config::Pgo::Occupancy options;
    options.enabled = true;
    options.resolution = 0.1;
    options.h_clearance = 0.15;
    options.clear_height_eps = 0.1;
    options.d_max = 1.5;
    options.occ_threshold = 0.3;
    options.usable_range = 20.0;
    options.min_range = 0.2;
    options.cloud_voxel_size = 0.0;
    options.margin = 2.0;

    sapphire::OccupancyGrid grid(options);
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();

    // Phase 1 — single wall point per scan, 2 scans → hit_cnt=2 at wall cell.
    // Low hit_cnt lets the clearing rays in Phase 2 dilute the ratio quickly.
    {
        auto cloud = std::make_shared<sapphire::PointCloud>();
        sapphire::Point w{}; w.x = 4.0f; w.y = 0.0f; w.z = 0.1f; w.data[3] = 1.0f;
        cloud->push_back(w);
        for (int i = 0; i < 2; ++i) grid.insertScan(cloud, pose);
    }
    expect(sampleCell(grid.toMsg(), 4.0, 0.0) == 100,
        "initial wall must be occupied");

    // Phase 2 — wall at x=4.2, same z. Free rays through the old cell have
    // d_ray ≈ 0.1*(40/41) ≈ 0.098 ≤ d_min(0.1)+eps(0.1)=0.2 → should clear.
    // Need ~20 rays to bring ratio 2/(2+N)<0.3 → N > 4.7. 12 scans × 1 ray.
    {
        auto cloud = std::make_shared<sapphire::PointCloud>();
        sapphire::Point w{}; w.x = 4.2f; w.y = 0.0f; w.z = 0.1f; w.data[3] = 1.0f;
        cloud->push_back(w);
        for (int i = 0; i < 12; ++i) grid.insertScan(cloud, pose);
    }

    const sapphire::OccupancyGridMsg msg = grid.toMsg();
    expect(sampleCell(msg, 4.2, 0.0) == 100,
        "jittered wall endpoint must be occupied");
    expect(sampleCell(msg, 4.0, 0.0) != 100,
        "previous wall cell must be cleared by same-height free rays");
}

void testObstacleClearsAfterDisappears() {
    // P0 regression: when an obstacle disappears, free rays through the cell
    // must increase visit_cnt so the hit/visit ratio eventually drops below
    // occ_threshold and the cell transitions from occupied to free.
    //
    // This exercises the d-aware clearing path: d_min is set to the wall
    // height, and same-height free rays (d_ray <= d_min + clear_height_eps)
    // must be allowed to increment visit_cnt.
    sapphire::Config::Pgo::Occupancy options;
    options.enabled = true;
    options.resolution = 0.1;
    options.h_clearance = 0.15;
    options.ground_margin = 0.0;
    options.d_max = 1.5;
    options.clear_height_eps = 0.05;
    options.occ_threshold = 0.3;
    options.usable_range = 20.0;
    options.min_range = 0.2;
    options.cloud_voxel_size = 0.0;
    options.margin = 2.0;

    sapphire::OccupancyGrid grid(options);
    sapphire::Isometry3d pose = sapphire::Isometry3d::Identity();

    const double wall_x = 4.0;
    const double far_x = 7.0;

    // Phase 1 — wall at x=4.0, d=0.5. Single point keeps hit_cnt low so the
    // ratio can decay below occ_threshold with a feasible number of free-ray scans.
    {
        auto cloud = std::make_shared<sapphire::PointCloud>();
        sapphire::Point point{};
        point.x = static_cast<float>(wall_x);
        point.y = 0.0f;
        point.z = 0.5f;
        point.data[3] = 1.0f;
        cloud->push_back(point);
        for (int s = 0; s < 4; ++s) {
            grid.insertScan(cloud, pose);
        }
    }
    expect(sampleCell(grid.toMsg(), wall_x, 0.0) == 100,
        "wall must be occupied");

    // Phase 2 — wall disappears. Rays go through to a far obstacle at x=7.0.
    // Free rays at the wall cell have d_ray ≈ 0.5 * (4/7) ≈ 0.29 which is
    // <= d_min(0.5) + eps(0.05) → should increment visit_cnt and dilute ratio.
    // hit_cnt=4 from Phase 1 → need visit_cnt > 4/0.3 ≈ 13.3 → ~10 scans.
    {
        auto cloud = std::make_shared<sapphire::PointCloud>();
        for (int i = 0; i < 4; ++i) {
            sapphire::Point point{};
            point.x = static_cast<float>(far_x);
            point.y = 0.0f;
            point.z = 0.5f;
            point.data[3] = 1.0f;
            cloud->push_back(point);
        }
        for (int s = 0; s < 10; ++s) {
            grid.insertScan(cloud, pose);
        }
    }

    const sapphire::OccupancyGridMsg msg = grid.toMsg();
    expect(sampleCell(msg, wall_x, 0.0) != 100,
        "wall cell must be cleared after obstacle disappears");
}

}  // namespace

int main() {
    try {
        testAttitudePlaneMarksWallOccupied();
        testGroundFreeRaysDoNotEraseWall();
        testCeilingFilteredByDMax();
        testGroundMarginSkipsNearGroundHits();
        testDepthJitterClearsPreviousWallCell();
                std::cout << "occupancy_grid_test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "occupancy_grid_test failed: " << error.what() << '\n';
        return 1;
    }
}
