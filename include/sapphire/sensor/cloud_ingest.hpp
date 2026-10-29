// PointCloud2 ingestion. Header-only; the ROS wrapper builds a CloudView and
// the core never parses sensor_msgs itself.
//
// Point::timestamp / CanonicalPoint::stamp are absolute seconds on the same
// clock as ImuData.stamp. Unit comes from the scan span, not the magnitude.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace sapphire::sensor {

struct FieldDescriptor {
    std::string name;
    uint32_t offset = 0;
    uint8_t datatype = 0;  // 1=INT8 2=UINT8 3=INT16 4=UINT16 5=INT32 6=UINT32 7=FLOAT32 8=FLOAT64
    uint32_t count = 1;
};

// height is derived from point_count and width. It is not stored on the view.
struct CloudView {
    std::vector<FieldDescriptor> fields;
    const uint8_t* data = nullptr;
    size_t data_size = 0;       // 0 = do not check the byte length
    size_t point_count = 0;
    size_t width = 0;           // 0 = packed, unorganized
    uint32_t point_step = 0;
    uint32_t row_step = 0;      // 0 = packed (no row padding)
    bool is_bigendian = false;
    double header_stamp = 0.0;  // seconds
};

struct CanonicalPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    double stamp = 0.0;  // absolute seconds, same clock as the IMU
};

enum class TimeUnit { Seconds, Milli, Micro, Nano };
enum class TimeRef { Relative, Absolute };

inline double unitToSeconds(TimeUnit unit) {
    switch (unit) {
        case TimeUnit::Seconds: return 1.0;
        case TimeUnit::Milli: return 1e-3;
        case TimeUnit::Micro: return 1e-6;
        case TimeUnit::Nano: return 1e-9;
    }
    return 1.0;
}

inline const char* unitName(TimeUnit unit) {
    switch (unit) {
        case TimeUnit::Seconds: return "s";
        case TimeUnit::Milli: return "ms";
        case TimeUnit::Micro: return "us";
        case TimeUnit::Nano: return "ns";
    }
    return "?";
}

// time_field / time_unit / time_ref default to automatic classification.
// require_point_time stays on sapphire::Config and is applied by the ROS
// wrapper, not by ingest().
struct IngestConfig {
    std::string time_field = "auto";
    std::string time_unit = "auto";
    std::string time_ref = "auto";
    // Unit window is [1e-3, max_scan_duration] seconds. 0.5 s keeps a raw
    // span of 1 from matching both seconds and milliseconds.
    double max_scan_duration = 0.5;
    double header_tolerance = 0.5;
};

struct TimeDecision {
    bool ok = false;
    std::string field;
    TimeUnit unit = TimeUnit::Seconds;
    TimeRef ref = TimeRef::Relative;
    double span_s = 0.0;
    double t_min_abs = 0.0;
    double t_max_abs = 0.0;
    double header_offset_s = 0.0;
    bool header_mismatch = false;
    std::string note;
};

struct IngestResult {
    std::vector<CanonicalPoint> points;
    TimeDecision time;
    size_t input_points = 0;
    size_t dropped_nonfinite = 0;
    size_t dropped_time = 0;
    bool deskewable = false;
};

namespace detail {

inline size_t typeSize(uint8_t datatype) {
    switch (datatype) {
        case 1:
        case 2: return 1;
        case 3:
        case 4: return 2;
        case 5:
        case 6:
        case 7: return 4;
        case 8: return 8;
        default: return 0;
    }
}

inline double readScalar(const uint8_t* p, uint8_t datatype, bool big_endian) {
    uint8_t buf[8];
    const size_t n = typeSize(datatype);
    if (n == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::memcpy(buf, p, n);
    if (big_endian) {
        std::reverse(buf, buf + n);
    }
    switch (datatype) {
        case 1: {
            int8_t v;
            std::memcpy(&v, buf, 1);
            return static_cast<double>(v);
        }
        case 2: {
            uint8_t v;
            std::memcpy(&v, buf, 1);
            return static_cast<double>(v);
        }
        case 3: {
            int16_t v;
            std::memcpy(&v, buf, 2);
            return static_cast<double>(v);
        }
        case 4: {
            uint16_t v;
            std::memcpy(&v, buf, 2);
            return static_cast<double>(v);
        }
        case 5: {
            int32_t v;
            std::memcpy(&v, buf, 4);
            return static_cast<double>(v);
        }
        case 6: {
            uint32_t v;
            std::memcpy(&v, buf, 4);
            return static_cast<double>(v);
        }
        case 7: {
            float v;
            std::memcpy(&v, buf, 4);
            return static_cast<double>(v);
        }
        case 8: {
            double v;
            std::memcpy(&v, buf, 8);
            return v;
        }
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

inline const uint8_t* pointPtr(const CloudView& view, size_t i) {
    if (view.row_step == 0 || view.width == 0
        || view.row_step == view.point_step * view.width) {
        return view.data + i * view.point_step;
    }
    return view.data + (i / view.width) * view.row_step
        + (i % view.width) * view.point_step;
}

inline bool inBounds(const CloudView& view, const FieldDescriptor& field) {
    const size_t size = typeSize(field.datatype);
    return size != 0
        && static_cast<size_t>(field.offset) + size <= view.point_step;
}

inline std::string layoutNote(const CloudView& view) {
    if (view.point_step == 0) {
        return "point_step is zero";
    }
    size_t height = 0;
    if (view.width > 0) {
        if (view.point_count % view.width != 0) {
            return "point_count is not divisible by width";
        }
        height = view.point_count / view.width;
        if (view.row_step != 0
            && static_cast<size_t>(view.row_step)
                < view.width * static_cast<size_t>(view.point_step)) {
            return "row_step is shorter than one row of points";
        }
    }
    if (view.data_size != 0) {
        const size_t need = (view.width == 0 || view.row_step == 0)
            ? view.point_count * static_cast<size_t>(view.point_step)
            : (height - 1) * static_cast<size_t>(view.row_step)
                + view.width * static_cast<size_t>(view.point_step);
        if (view.data_size < need) {
            return "point cloud data is shorter than the last point";
        }
    }
    return {};
}

}  // namespace detail

inline const std::vector<std::string>& timeFieldCandidates() {
    // Wrapper order first. time_stamp is an extra alias and must not jump
    // ahead of timestamp or t.
    static const std::vector<std::string> names = {
        "offset_time", "t", "timestamp", "time", "time_stamp"};
    return names;
}

inline const std::vector<std::string>& intensityFieldCandidates() {
    static const std::vector<std::string> names = {
        "intensity", "reflectivity", "reflectance", "i", "refl"};
    return names;
}

inline const FieldDescriptor* findField(
    const CloudView& view, const std::string& name)
{
    for (const auto& field : view.fields) {
        if (field.name == name) {
            return &field;
        }
    }
    return nullptr;
}

inline const FieldDescriptor* findFirst(
    const CloudView& view,
    const std::vector<std::string>& names,
    const std::string& forced = "auto")
{
    if (forced != "auto") {
        return findField(view, forced);
    }
    for (const auto& name : names) {
        if (const FieldDescriptor* field = findField(view, name)) {
            return field;
        }
    }
    return nullptr;
}

// A scan lasts between 1 ms and max_scan_duration. Exactly one of
// {s, ms, us, ns} must land in that window. More than one candidate is a
// failure; the largest unit is not preferred.
inline TimeDecision classifyTime(
    double raw_min,
    double raw_max,
    double header_stamp,
    const IngestConfig& cfg,
    const std::string& field_name)
{
    TimeDecision decision;
    decision.field = field_name;

    if (!std::isfinite(raw_min) || !std::isfinite(raw_max)) {
        decision.note = "time field contains non-finite values";
        return decision;
    }
    const double span_raw = raw_max - raw_min;
    if (span_raw < 1e-12) {
        decision.note = "point timestamps are identical (no intra-scan time)";
        return decision;
    }

    if (cfg.time_unit != "auto") {
        if (cfg.time_unit == "s") {
            decision.unit = TimeUnit::Seconds;
        } else if (cfg.time_unit == "ms") {
            decision.unit = TimeUnit::Milli;
        } else if (cfg.time_unit == "us") {
            decision.unit = TimeUnit::Micro;
        } else if (cfg.time_unit == "ns") {
            decision.unit = TimeUnit::Nano;
        } else {
            decision.note = "time_unit must be auto|s|ms|us|ns";
            return decision;
        }
    } else {
        struct Hit {
            TimeUnit unit;
            double span_s;
        };
        std::vector<Hit> hits;
        const TimeUnit units[] = {
            TimeUnit::Seconds,
            TimeUnit::Milli,
            TimeUnit::Micro,
            TimeUnit::Nano,
        };
        for (TimeUnit unit : units) {
            const double span_s = span_raw * unitToSeconds(unit);
            if (span_s >= 1e-3 && span_s <= cfg.max_scan_duration) {
                hits.push_back(Hit{unit, span_s});
            }
        }
        if (hits.size() != 1) {
            std::ostringstream note;
            if (hits.empty()) {
                note << "unable to infer time unit from span_raw=" << span_raw
                     << "; converted:";
                for (TimeUnit unit : units) {
                    note << ' ' << unitName(unit) << '='
                         << (span_raw * unitToSeconds(unit)) << 's';
                }
            } else {
                note << "ambiguous time unit:";
                for (const Hit& hit : hits) {
                    note << ' ' << unitName(hit.unit) << '=' << hit.span_s
                         << 's';
                }
                note << " (span_raw=" << span_raw << ")";
            }
            decision.note = note.str();
            return decision;
        }
        decision.unit = hits[0].unit;
    }

    const double scale = unitToSeconds(decision.unit);
    const double span_s = span_raw * scale;
    decision.span_s = span_s;

    const double tol = std::max(2.0 * span_s, 0.05);
    const double raw_min_s = raw_min * scale;
    bool absolute = false;
    if (cfg.time_ref != "auto") {
        if (cfg.time_ref == "absolute") {
            absolute = true;
        } else if (cfg.time_ref == "relative") {
            absolute = false;
        } else {
            decision.note = "time_ref must be auto|relative|absolute";
            return decision;
        }
    } else if (std::abs(raw_min_s - header_stamp) <= tol) {
        absolute = true;
    } else if (std::abs(raw_min_s) <= tol) {
        absolute = false;
    } else if (std::abs(raw_min_s) > 1e6) {
        absolute = true;
    } else if (std::abs(header_stamp) < 1.0) {
        absolute = false;
    } else {
        std::ostringstream note;
        note << "unable to decide absolute or relative time: min=" << raw_min_s
             << " span=" << span_s << " header=" << header_stamp;
        decision.note = note.str();
        return decision;
    }
    decision.ref = absolute ? TimeRef::Absolute : TimeRef::Relative;

    decision.t_min_abs = absolute ? raw_min_s : header_stamp + raw_min_s;
    decision.t_max_abs =
        absolute ? raw_max * scale : header_stamp + raw_max * scale;
    decision.header_offset_s = decision.t_min_abs - header_stamp;
    decision.header_mismatch =
        std::abs(decision.header_offset_s) > cfg.header_tolerance;
    if (decision.header_mismatch) {
        std::ostringstream note;
        note << "point time and header differ by " << decision.header_offset_s
             << " s (different clock or epoch); using point time";
        decision.note = note.str();
    }
    decision.ok = true;
    return decision;
}

inline IngestResult ingest(const CloudView& view, const IngestConfig& cfg) {
    IngestResult out;
    out.input_points = view.point_count;

    const FieldDescriptor* fx = findField(view, "x");
    const FieldDescriptor* fy = findField(view, "y");
    const FieldDescriptor* fz = findField(view, "z");
    if (!fx || !fy || !fz || view.data == nullptr || view.point_count == 0) {
        out.time.note = "missing x/y/z or empty cloud";
        return out;
    }
    const std::string layout = detail::layoutNote(view);
    if (!layout.empty()) {
        out.time.note = layout;
        return out;
    }
    if (!detail::inBounds(view, *fx) || !detail::inBounds(view, *fy)
        || !detail::inBounds(view, *fz)) {
        out.time.note = "x/y/z field exceeds point_step";
        return out;
    }

    const FieldDescriptor* fi =
        findFirst(view, intensityFieldCandidates(), "auto");
    if (fi && !detail::inBounds(view, *fi)) {
        out.time.note = "intensity field '" + fi->name + "' exceeds point_step";
        return out;
    }

    const FieldDescriptor* ft = nullptr;
    if (cfg.time_field != "none") {
        ft = findFirst(view, timeFieldCandidates(), cfg.time_field);
    }

    std::vector<double> raw_times;
    if (ft) {
        if (!detail::inBounds(view, *ft)) {
            out.time.note = "time field '" + ft->name + "' exceeds point_step";
            return out;
        }
        if (detail::typeSize(ft->datatype) == 0) {
            out.time.field = ft->name;
            out.time.note = "unsupported time field type";
        } else {
            raw_times.resize(view.point_count);
            double tmin = std::numeric_limits<double>::infinity();
            double tmax = -std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < view.point_count; ++i) {
                const double sample = detail::readScalar(
                    detail::pointPtr(view, i) + ft->offset,
                    ft->datatype,
                    view.is_bigendian);
                raw_times[i] = sample;
                if (std::isfinite(sample)) {
                    tmin = std::min(tmin, sample);
                    tmax = std::max(tmax, sample);
                }
            }
            if (std::isfinite(tmin)) {
                out.time = classifyTime(
                    tmin, tmax, view.header_stamp, cfg, ft->name);
            } else {
                out.time.field = ft->name;
                out.time.note = "time field is entirely non-finite";
            }
        }
    } else if (cfg.time_field != "auto" && cfg.time_field != "none") {
        out.time.note = "time field '" + cfg.time_field + "' was not found";
    } else if (cfg.time_field == "none") {
        out.time.note = "time field disabled";
    } else {
        out.time.note = "point cloud has no recognizable time field";
    }

    const bool have_time = out.time.ok;
    const double scale = unitToSeconds(out.time.unit);
    const double t_off =
        (out.time.ref == TimeRef::Relative) ? view.header_stamp : 0.0;

    auto emit = [&](double x, double y, double z, float intensity, size_t i) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            ++out.dropped_nonfinite;
            return;
        }
        // Non-finite times are dropped only after a successful classification.
        // A failed classification stamps every finite xyz point with the header.
        if (have_time && !std::isfinite(raw_times[i])) {
            ++out.dropped_time;
            return;
        }
        CanonicalPoint point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);
        point.intensity = intensity;
        point.stamp = have_time
            ? t_off + raw_times[i] * scale
            : view.header_stamp;
        out.points.push_back(point);
    };

    out.points.reserve(view.point_count);
    const bool float_xyz = fx->datatype == 7 && fy->datatype == 7
        && fz->datatype == 7 && (fi == nullptr || fi->datatype == 7);
    // Big-endian fields go through readScalar, which swaps bytes. A host
    // memcpy of the raw column would keep the wire order.
    if (!view.is_bigendian && float_xyz) {
        for (size_t i = 0; i < view.point_count; ++i) {
            const uint8_t* raw = detail::pointPtr(view, i);
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::memcpy(&x, raw + fx->offset, 4);
            std::memcpy(&y, raw + fy->offset, 4);
            std::memcpy(&z, raw + fz->offset, 4);
            float intensity = 0.0f;
            if (fi != nullptr) {
                std::memcpy(&intensity, raw + fi->offset, 4);
            }
            emit(x, y, z, intensity, i);
        }
    } else {
        for (size_t i = 0; i < view.point_count; ++i) {
            const uint8_t* raw = detail::pointPtr(view, i);
            const float intensity = fi == nullptr
                ? 0.0f
                : static_cast<float>(detail::readScalar(
                      raw + fi->offset, fi->datatype, view.is_bigendian));
            emit(detail::readScalar(raw + fx->offset, fx->datatype, view.is_bigendian),
                 detail::readScalar(raw + fy->offset, fy->datatype, view.is_bigendian),
                 detail::readScalar(raw + fz->offset, fz->datatype, view.is_bigendian),
                 intensity,
                 i);
        }
    }

    out.deskewable = out.time.ok && out.points.size() > 1;
    return out;
}

}  // namespace sapphire::sensor
