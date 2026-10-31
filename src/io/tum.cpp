#include <sapphire/io/tum.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sapphire {
namespace {

std::string tumLine(
    double stamp,
    const Eigen::Vector3d& translation,
    const Eigen::Quaterniond& rotation)
{
    std::ostringstream line;
    line.imbue(std::locale::classic());
    line << std::fixed << std::setprecision(9)
         << stamp << ' '
         << translation.x() << ' '
         << translation.y() << ' '
         << translation.z() << ' '
         << rotation.x() << ' '
         << rotation.y() << ' '
         << rotation.z() << ' '
         << rotation.w() << '\n';
    return line.str();
}

}  // namespace

TumWriteResult writeTumTrajectory(
    const std::filesystem::path& path,
    const MapTrajectory& trajectory)
{
    TumWriteResult result;
    std::string body;
    double previous_stamp = -std::numeric_limits<double>::infinity();
    bool have_stamp = false;

    for (const MapTrajectorySample& sample : trajectory.samples) {
        const Eigen::Vector3d translation = sample.T_map_lidar.translation();
        const Eigen::Quaterniond rotation(sample.T_map_lidar.rotation());
        const bool finite = std::isfinite(sample.stamp)
            && translation.allFinite()
            && std::isfinite(rotation.x())
            && std::isfinite(rotation.y())
            && std::isfinite(rotation.z())
            && std::isfinite(rotation.w());
        const bool increasing = !have_stamp || sample.stamp > previous_stamp;
        if (!finite || !increasing) {
            ++result.skipped;
            continue;
        }
        body += tumLine(sample.stamp, translation, rotation);
        previous_stamp = sample.stamp;
        have_stamp = true;
        ++result.written;
    }

    if (result.written == 0) {
        return result;
    }

    std::error_code error;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            throw std::runtime_error(
                "failed to create trajectory directory '"
                + path.parent_path().string() + "': " + error.message());
        }
    }

    const std::filesystem::path temporary =
        path.parent_path() / (path.filename().string() + ".tmp");
    {
        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
        if (!stream) {
            throw std::runtime_error(
                "failed to open trajectory file '" + temporary.string() + "'");
        }
        stream << body;
        stream.flush();
        if (!stream) {
            std::filesystem::remove(temporary);
            throw std::runtime_error(
                "failed to write trajectory file '" + temporary.string() + "'");
        }
    }

    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::filesystem::remove(temporary);
        throw std::runtime_error(
            "failed to publish trajectory file '" + path.string()
            + "': " + error.message());
    }
    return result;
}

}  // namespace sapphire
