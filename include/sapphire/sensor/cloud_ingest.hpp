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

inline const char* refName(TimeRef ref) {
    return ref == TimeRef::Absolute ? "absolute" : "relative";
}

inline bool parseTimeUnit(const std::string& name, TimeUnit& unit) {
    if (name == "s") {
        unit = TimeUnit::Seconds;
        return true;
    }
    if (name == "ms") {
        unit = TimeUnit::Milli;
        return true;
    }
    if (name == "us") {
        unit = TimeUnit::Micro;
        return true;
    }
    if (name == "ns") {
        unit = TimeUnit::Nano;
        return true;
    }
    return false;
}

inline bool parseTimeRef(const std::string& name, TimeRef& ref) {
    if (name == "relative") {
        ref = TimeRef::Relative;
        return true;
    }
    if (name == "absolute") {
        ref = TimeRef::Absolute;
        return true;
    }
    return false;
}

// time_field / time_unit / time_ref default to automatic classification.
// Defaults match Config::Sensor::Lidar. require_point_time is not an ingest
// input; LidarTimeSession applies it when a frame is committed.
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
    const size_t step = view.point_step;
    size_t height = 0;
    if (view.width > 0) {
        if (view.point_count % view.width != 0) {
            return "point_count is not divisible by width";
        }
        height = view.point_count / view.width;
        if (view.row_step != 0
            && view.width > static_cast<size_t>(view.row_step) / step) {
            return "row_step is shorter than one row of points";
        }
    }
    // data_size == 0 skips the byte check. Division keeps a wrapping product
    // from comparing as a small length and then throwing in reserve().
    if (view.data_size == 0) {
        return {};
    }
    if (view.width == 0 || view.row_step == 0 || height == 0) {
        if (view.point_count > view.data_size / step) {
            return "point cloud data is shorter than the last point";
        }
        return {};
    }
    const size_t row_step = view.row_step;
    size_t used = 0;
    if (height > 1) {
        if (height - 1 > view.data_size / row_step) {
            return "point cloud data is shorter than the last point";
        }
        used = (height - 1) * row_step;
    }
    if (view.width * step > view.data_size - used) {
        return "point cloud data is shorter than the last point";
    }
    return {};
}

}  // namespace detail

// Search order is the wrapper order. time_stamp is an extra alias and must
// not jump ahead of timestamp or t. "auto" and "none" are policy words, not
// field names. TOML validation uses this same list.
inline const char* const* timeFieldChoices() {
    static const char* const names[] = {
        "auto",
        "none",
        "offset_time",
        "t",
        "timestamp",
        "time",
        "time_stamp",
        nullptr,
    };
    return names;
}

inline const char* const* timeUnitChoices() {
    static const char* const names[] = {
        "auto", "s", "ms", "us", "ns", nullptr};
    return names;
}

inline const char* const* timeRefChoices() {
    static const char* const names[] = {
        "auto", "relative", "absolute", nullptr};
    return names;
}

inline const std::vector<std::string>& timeFieldCandidates() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> values;
        for (const char* const* item = timeFieldChoices();
             *item != nullptr;
             ++item) {
            const std::string name(*item);
            if (name != "auto" && name != "none") {
                values.push_back(name);
            }
        }
        return values;
    }();
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
        if (!parseTimeUnit(cfg.time_unit, decision.unit)) {
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
        TimeRef forced = TimeRef::Relative;
        if (!parseTimeRef(cfg.time_ref, forced)) {
            decision.note = "time_ref must be auto|relative|absolute";
            return decision;
        }
        absolute = forced == TimeRef::Absolute;
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

// Owns the lidar time policy for one stream: the ingest config, whether a
// failed classification drops the frame, and the lock taken from the first
// successful classification. Later scans reuse that field, unit, and
// reference. Callers still turn sensor messages into CloudView and print
// the Commit flags. ingest() does not mutate the session. commit() is not
// thread-safe; the caller serializes frames. A non-finite header stamp is
// the caller's check and must not be committed.
class LidarTimeSession {
public:
    struct Commit {
        bool accept = false;
        std::string drop_reason;
        bool just_locked = false;
        bool warn_span_ratio = false;
        bool warn_header_mismatch = false;
        double locked_span_s = 0.0;
        double span_s = 0.0;
        double header_offset_s = 0.0;
    };

    LidarTimeSession() = default;

    LidarTimeSession(IngestConfig config, bool require_point_time)
        : config_(std::move(config))
        , require_point_time_(require_point_time) {}

    const IngestConfig& config() const { return config_; }
    bool requirePointTime() const { return require_point_time_; }
    bool locked() const { return locked_; }

    IngestResult ingest(const CloudView& view) const {
        return sapphire::sensor::ingest(view, config_);
    }

    Commit commit(const IngestResult& ingested) {
        Commit decision;
        if (ingested.points.empty()) {
            decision.drop_reason = ingested.time.note.empty()
                ? "no points"
                : ingested.time.note;
            return decision;
        }
        if (!ingested.time.ok && require_point_time_) {
            decision.drop_reason = ingested.time.note;
            return decision;
        }

        decision.accept = true;
        decision.span_s = ingested.time.span_s;
        decision.header_offset_s = ingested.time.header_offset_s;
        decision.locked_span_s = locked_span_s_;
        if (ingested.time.ok && !locked_) {
            config_.time_field = ingested.time.field;
            config_.time_unit = unitName(ingested.time.unit);
            config_.time_ref = refName(ingested.time.ref);
            locked_span_s_ = ingested.time.span_s;
            locked_ = true;
            decision.just_locked = true;
            decision.locked_span_s = locked_span_s_;
        } else if (
            ingested.time.ok && locked_ && locked_span_s_ > 0.0
            && !warned_span_ratio_) {
            const double ratio = ingested.time.span_s / locked_span_s_;
            if (ratio > 10.0 || ratio < 0.1) {
                warned_span_ratio_ = true;
                decision.warn_span_ratio = true;
            }
        }
        if (
            ingested.time.ok && ingested.time.header_mismatch
            && !warned_header_mismatch_) {
            warned_header_mismatch_ = true;
            decision.warn_header_mismatch = true;
        }
        return decision;
    }

private:
    IngestConfig config_;
    bool require_point_time_ = false;
    bool locked_ = false;
    double locked_span_s_ = 0.0;
    bool warned_span_ratio_ = false;
    bool warned_header_mismatch_ = false;
};

}  // namespace sapphire::sensor
