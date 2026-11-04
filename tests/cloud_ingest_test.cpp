#include <sapphire/sensor/cloud_ingest.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <vector>

using namespace sapphire::sensor;

struct Builder {
    struct F {
        std::string name;
        uint8_t type;
        uint32_t offset;
    };
    std::vector<F> fields;
    std::vector<uint8_t> data;
    uint32_t step = 0;
    size_t n = 0;
    double header = 0.0;
    bool bigendian = false;
    size_t data_size = 0;
    size_t width = 0;
    uint32_t row_step = 0;

    void add(const std::string& name, uint8_t type) {
        fields.push_back({name, type, step});
        step += [&] {
            switch (type) {
                case 1:
                case 2: return 1u;
                case 3:
                case 4: return 2u;
                case 5:
                case 6:
                case 7: return 4u;
                default: return 8u;
            }
        }();
    }

    void alloc(size_t count) {
        n = count;
        data.assign(n * step, 0);
    }

    void set(size_t i, const std::string& name, double v) {
        for (const auto& field : fields) {
            if (field.name != name) {
                continue;
            }
            uint8_t* p = data.data() + i * step + field.offset;
            switch (field.type) {
                case 2: {
                    uint8_t x = static_cast<uint8_t>(v);
                    std::memcpy(p, &x, 1);
                    break;
                }
                case 4: {
                    uint16_t x = static_cast<uint16_t>(v);
                    std::memcpy(p, &x, 2);
                    break;
                }
                case 6: {
                    uint32_t x = static_cast<uint32_t>(v);
                    std::memcpy(p, &x, 4);
                    break;
                }
                case 7: {
                    float x = static_cast<float>(v);
                    std::memcpy(p, &x, 4);
                    break;
                }
                case 8: {
                    double x = v;
                    std::memcpy(p, &x, 8);
                    break;
                }
                default:
                    break;
            }
            return;
        }
    }

    void xyz(size_t i, double x, double y, double z, double intensity = 10.0) {
        set(i, "x", x);
        set(i, "y", y);
        set(i, "z", z);
        if (has("intensity")) {
            set(i, "intensity", intensity);
        }
    }

    bool has(const std::string& name) const {
        for (const auto& field : fields) {
            if (field.name == name) {
                return true;
            }
        }
        return false;
    }

    void swapEndian() {
        for (size_t i = 0; i < n; ++i) {
            for (const auto& field : fields) {
                size_t size = 1;
                switch (field.type) {
                    case 1:
                    case 2: size = 1; break;
                    case 3:
                    case 4: size = 2; break;
                    case 5:
                    case 6:
                    case 7: size = 4; break;
                    default: size = 8; break;
                }
                uint8_t* p = data.data() + i * step + field.offset;
                std::reverse(p, p + size);
            }
        }
        bigendian = true;
    }

    CloudView view() const {
        CloudView cloud;
        for (const auto& field : fields) {
            cloud.fields.push_back({field.name, field.offset, field.type, 1});
        }
        cloud.data = data.data();
        cloud.data_size = data_size;
        cloud.point_count = n;
        cloud.width = width;
        cloud.point_step = step;
        cloud.row_step = row_step;
        cloud.is_bigendian = bigendian;
        cloud.header_stamp = header;
        return cloud;
    }
};

int failures = 0;

void check(bool cond, const std::string& what) {
    if (!cond) {
        std::printf("    FAIL: %s\n", what.c_str());
        ++failures;
    }
}

void checkRejectedLayout(
    const CloudView& cloud, const IngestConfig& cfg, const char* name)
{
    bool threw = false;
    IngestResult result;
    try {
        result = ingest(cloud, cfg);
    } catch (const std::exception&) {
        threw = true;
    }
    check(!threw, std::string(name) + ": does not throw");
    if (threw) {
        return;
    }
    check(!result.time.ok, std::string(name) + ": rejected");
    check(result.points.empty(), std::string(name) + ": yields no points");
    check(
        result.time.note.find("shorter") != std::string::npos,
        std::string(name) + ": reports a short buffer");
}

void report(
    const char* name,
    const IngestResult& result,
    TimeUnit unit,
    TimeRef ref,
    double span,
    bool expect_ok = true)
{
    const auto& time = result.time;
    std::printf(
        "  %-34s ok=%d field=%-11s unit=%-2s ref=%-8s span=%.6fs "
        "hdr_off=%+.6fs pts=%zu\n",
        name,
        static_cast<int>(time.ok),
        time.field.c_str(),
        time.ok ? unitName(time.unit) : "-",
        time.ok ? (time.ref == TimeRef::Absolute ? "absolute" : "relative")
                : "-",
        time.span_s,
        time.header_offset_s,
        result.points.size());
    if (!time.note.empty()) {
        std::printf("       note: %s\n", time.note.c_str());
    }
    if (expect_ok) {
        check(time.ok, std::string(name) + ": should classify");
        check(time.unit == unit, std::string(name) + ": unit");
        check(time.ref == ref, std::string(name) + ": reference");
        check(std::abs(time.span_s - span) < 1e-3, std::string(name) + ": span");
        check(result.deskewable, std::string(name) + ": deskewable");
    } else {
        check(!time.ok, std::string(name) + ": should NOT classify");
        check(!result.deskewable, std::string(name) + ": not deskewable");
    }
}

constexpr double kHeader = 1779083760.4001343;

int main() {
    const size_t N = 1000;
    IngestConfig cfg;
    check(cfg.max_scan_duration == 0.5, "default max_scan_duration is 0.5 s");

    std::printf("== 1. Livox Mid-360 PointCloud2\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("tag", 2);
        b.add("line", 2);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            if (i == 0) {
                b.xyz(i, 0.0, 0.0, 0.0);
            } else {
                b.xyz(i, 5.0, 1.0, 2.0);
            }
            b.set(i, "timestamp", 99829965.0 * static_cast<double>(i) / (N - 1));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "livox_mid360/rel_ns_float64",
            result,
            TimeUnit::Nano,
            TimeRef::Relative,
            0.099829965);
        check(
            std::abs(result.time.t_min_abs - kHeader) < 1e-6,
            "livox: t_min == header");
        check(result.points.size() == N, "livox: origin return is kept");
        check(
            result.points.size() == N && result.points.front().x == 0.0f,
            "livox: no range gate");
    }

    std::printf("== 2. Livox legacy offset_time UINT32\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("offset_time", 6);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "offset_time", 100000000.0 * static_cast<double>(i) / (N - 1));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "livox_legacy/rel_ns_uint32",
            result,
            TimeUnit::Nano,
            TimeRef::Relative,
            0.1);
    }

    std::printf("== 3. Hesai absolute unix seconds\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("ring", 4);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", kHeader + 0.0001 * static_cast<double>(i));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "hesai/abs_s_float64",
            result,
            TimeUnit::Seconds,
            TimeRef::Absolute,
            0.0999);
        check(
            std::abs(result.time.t_min_abs - kHeader) < 1e-6,
            "hesai: t_min == header");
    }

    std::printf("== 4. RoboSense absolute unix seconds\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("ring", 4);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", kHeader + 0.0001 * static_cast<double>(i));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "robosense/abs_s_float64",
            result,
            TimeUnit::Seconds,
            TimeRef::Absolute,
            0.0999);
    }

    std::printf("== 5. Velodyne signed relative seconds\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("ring", 4);
        b.add("time", 7);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "time", -0.002 + 0.1 * static_cast<double>(i) / (N - 1));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "velodyne/rel_s_float32_signed",
            result,
            TimeUnit::Seconds,
            TimeRef::Relative,
            0.1);
        check(
            std::abs(result.time.t_min_abs - (kHeader - 0.002)) < 1e-6,
            "velodyne: t_min == header-2ms");
    }

    std::printf("== 6. Ouster t UINT32 relative nanoseconds\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("t", 6);
        b.add("reflectivity", 4);
        b.add("ring", 2);
        b.add("ambient", 4);
        b.add("range", 6);
        b.step = 32;
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "t", 100000000.0 * static_cast<double>(i) / (N - 1));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "ouster/rel_ns_uint32",
            result,
            TimeUnit::Nano,
            TimeRef::Relative,
            0.1);
    }

    std::printf("== 7. millisecond / microsecond fields\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("time", 6);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "time", 100.0 * static_cast<double>(i) / (N - 1));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "generic/rel_ms_uint32",
            result,
            TimeUnit::Milli,
            TimeRef::Relative,
            0.1);
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", kHeader * 1e6 + 100.0 * static_cast<double>(i));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "generic/abs_us_float64",
            result,
            TimeUnit::Micro,
            TimeRef::Absolute,
            0.0999);
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", kHeader * 1e9 + 1e5 * static_cast<double>(i));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "generic/abs_ns_float64",
            result,
            TimeUnit::Nano,
            TimeRef::Absolute,
            0.0999);
    }

    std::printf("== 8. degenerate inputs\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
        }
        auto result = ingest(b.view(), cfg);
        report(
            "no_time_field",
            result,
            TimeUnit::Seconds,
            TimeRef::Relative,
            0.0,
            false);
        check(result.points.size() == N, "no_time_field: points still delivered");
        check(
            result.points.size() == N
                && std::abs(result.points.front().stamp - kHeader) < 1e-9,
            "no_time_field: stamp is the header");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", 0.0);
        }
        auto result = ingest(b.view(), cfg);
        report(
            "constant_timestamps",
            result,
            TimeUnit::Seconds,
            TimeRef::Relative,
            0.0,
            false);
        check(result.points.size() == N, "constant timestamps: points kept");
        check(
            result.dropped_time == 0,
            "constant timestamps: dropped_time stays 0");
        check(
            result.points.size() == N
                && std::abs(result.points.front().stamp - kHeader) < 1e-9,
            "constant timestamps: stamp is the header");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("t", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "t", 1.5e12 + 1e5 * static_cast<double>(i));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "uptime_ns_ambiguous",
            result,
            TimeUnit::Nano,
            TimeRef::Absolute,
            0.0,
            false);
        check(result.points.size() == N, "uptime: points still delivered");
        check(result.time.note.find("min=") != std::string::npos, "uptime: note has min");
        check(
            result.time.note.find("span=") != std::string::npos,
            "uptime: note has span");
        check(
            result.time.note.find("header=") != std::string::npos,
            "uptime: note has header");
        const auto min_pos = result.time.note.find("min=");
        const double noted_min = min_pos == std::string::npos
            ? std::numeric_limits<double>::quiet_NaN()
            : std::strtod(result.time.note.c_str() + min_pos + 4, nullptr);
        check(std::abs(noted_min - 1500.0) < 1.0, "uptime: min is about 1500 s");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", kHeader + 1e8 + 1e-4 * static_cast<double>(i));
        }
        auto result = ingest(b.view(), cfg);
        report(
            "absolute_wrong_epoch",
            result,
            TimeUnit::Seconds,
            TimeRef::Absolute,
            0.0999);
        check(result.time.header_mismatch, "wrong epoch: mismatch is flagged");
        check(
            std::abs(result.time.header_offset_s - 1e8) < 1.0,
            "wrong epoch: offset is visible");
    }

    std::printf("== 9. organized cloud / NaN time / non-finite column\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("tag", 2);
        b.add("line", 2);
        b.add("timestamp", 8);
        const size_t w = 4;
        const size_t h = 2;
        const uint32_t row_step = b.step * static_cast<uint32_t>(w) + 8;
        b.n = w * h;
        b.data.assign(static_cast<size_t>(row_step) * h, 0);
        b.header = kHeader;
        b.width = w;
        b.row_step = row_step;
        b.data_size = b.data.size();
        for (size_t row = 0; row < h; ++row) {
            for (size_t col = 0; col < w; ++col) {
                const size_t i = row * w + col;
                uint8_t* base = b.data.data() + row * row_step + col * b.step;
                const float xyz[4] = {
                    static_cast<float>(100 * row + col), 1.0f, 2.0f, 10.0f};
                std::memcpy(base, xyz, 16);
                const double t = 1e6 * static_cast<double>(i);
                std::memcpy(base + 18, &t, 8);
            }
        }
        auto result = ingest(b.view(), cfg);
        check(result.time.ok, "organized: classifies");
        check(result.points.size() == w * h, "organized: all points kept");
        check(
            result.points.size() == w * h && result.points[4].x == 100.0f,
            "organized: row 1 read through row_step padding");
        check(
            result.points.size() == w * h && result.points[7].x == 103.0f,
            "organized: last point of row 1");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            if (i == N / 2) {
                b.set(i, "timestamp", std::numeric_limits<double>::quiet_NaN());
            } else {
                b.set(
                    i,
                    "timestamp",
                    1e8 * static_cast<double>(i) / static_cast<double>(N - 1));
            }
        }
        auto result = ingest(b.view(), cfg);
        check(result.time.ok, "NaN timestamp: classification succeeds");
        check(result.dropped_time == 1, "NaN timestamp: dropped");
        bool any_nan = false;
        for (const auto& point : result.points) {
            if (!std::isfinite(point.stamp)) {
                any_nan = true;
            }
        }
        check(!any_nan, "NaN timestamp: no NaN stamp reaches the core");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", std::numeric_limits<double>::quiet_NaN());
        }
        auto result = ingest(b.view(), cfg);
        check(!result.time.ok, "all-NaN time: classification fails");
        check(result.dropped_time == 0, "all-NaN time: dropped_time stays 0");
        check(result.points.size() == N, "all-NaN time: xyz points kept");
        check(
            result.points.size() == N
                && std::abs(result.points.front().stamp - kHeader) < 1e-9,
            "all-NaN time: stamp is the header");
    }

    std::printf("== 10. unit window\n");
    {
        IngestConfig wide = cfg;
        wide.max_scan_duration = 2.0;
        const TimeDecision decision =
            classifyTime(0.0, 1.5, kHeader, wide, "time");
        check(!decision.ok, "span_raw 1.5 with 2 s window is ambiguous");
        check(
            decision.note.find(" s=") != std::string::npos,
            "ambiguous unit note lists seconds");
        check(
            decision.note.find(" ms=") != std::string::npos,
            "ambiguous unit note lists milliseconds");
        std::printf("       note: %s\n", decision.note.c_str());
    }
    {
        const TimeDecision decision = classifyTime(0.0, 1.0, kHeader, cfg, "time");
        check(decision.ok, "span_raw 1 classifies under the 0.5 s window");
        check(decision.unit == TimeUnit::Milli, "span_raw 1 is milliseconds");
    }
    {
        struct Case {
            double span_raw;
            TimeUnit unit;
            const char* name;
        };
        const Case cases[] = {
            {0.1, TimeUnit::Seconds, "0.1 s"},
            {100.0, TimeUnit::Milli, "100 ms"},
            {1e5, TimeUnit::Micro, "1e5 us"},
            {1e8, TimeUnit::Nano, "1e8 ns"},
        };
        for (const Case& item : cases) {
            const TimeDecision decision =
                classifyTime(0.0, item.span_raw, kHeader, cfg, "time");
            check(decision.ok, std::string(item.name) + " classifies");
            check(
                decision.unit == item.unit,
                std::string(item.name) + " has one unit");
        }
    }

    std::printf("== 11. field order, endianness, layout\n");
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.add("t", 6);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(i, "timestamp", kHeader + 0.0001 * static_cast<double>(i));
            b.set(i, "t", 1e8 * static_cast<double>(i) / (N - 1));
        }
        auto result = ingest(b.view(), cfg);
        check(result.time.ok, "field order: classifies");
        check(result.time.field == "t", "field order: t precedes timestamp");
        check(result.time.unit == TimeUnit::Nano, "field order: t is nanoseconds");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("time", 7);
        b.alloc(8);
        b.header = kHeader;
        for (size_t i = 0; i < 8; ++i) {
            b.xyz(i, 5.25, -1.5, 2.5, 42.0);
            b.set(i, "time", 0.01 * static_cast<double>(i));
        }
        const float host = 5.25f;
        uint8_t little[4];
        std::memcpy(little, &host, 4);
        b.swapEndian();
        check(
            b.data[0] != little[0],
            "big-endian xyz fixture differs from host bytes");
        auto result = ingest(b.view(), cfg);
        check(result.time.ok, "big-endian xyz: classifies");
        check(result.points.size() == 8, "big-endian xyz: points kept");
        check(
            result.points.size() == 8
                && std::abs(result.points[3].x - 5.25f) < 1e-5f
                && std::abs(result.points[3].y - (-1.5f)) < 1e-5f
                && std::abs(result.points[3].z - 2.5f) < 1e-5f,
            "big-endian xyz: coordinates decoded");
        check(
            result.points.size() == 8
                && std::abs(result.points[3].intensity - 42.0f) < 1e-4f,
            "big-endian intensity decoded");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("intensity", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 5.0, 1.0, 2.0);
            b.set(
                i,
                "timestamp",
                1e8 * static_cast<double>(i) / static_cast<double>(N - 1));
        }
        b.swapEndian();
        auto result = ingest(b.view(), cfg);
        report(
            "bigendian/rel_ns_float64",
            result,
            TimeUnit::Nano,
            TimeRef::Relative,
            0.1);
        check(
            std::abs(result.time.t_min_abs - kHeader) < 1e-6,
            "big-endian time: t_min == header");
        check(
            result.points.size() == N
                && std::abs(result.points.back().stamp - (kHeader + 0.1)) < 1e-4,
            "big-endian time: last stamp");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("timestamp", 8);
        b.alloc(4);
        b.header = kHeader;
        for (size_t i = 0; i < 4; ++i) {
            b.xyz(i, 1.0, 2.0, 3.0);
            b.set(i, "timestamp", 1e7 * static_cast<double>(i));
        }
        CloudView cloud = b.view();
        for (auto& field : cloud.fields) {
            if (field.name == "timestamp") {
                field.offset = cloud.point_step;
            }
        }
        auto result = ingest(cloud, cfg);
        check(!result.time.ok, "time field past point_step fails");
        check(result.points.empty(), "time field past point_step yields no points");
        check(
            result.time.note.find("timestamp") != std::string::npos,
            "time field past point_step names the field");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("timestamp", 8);
        b.alloc(N);
        b.header = kHeader;
        for (size_t i = 0; i < N; ++i) {
            b.xyz(i, 1.0, 2.0, 3.0);
            b.set(i, "timestamp", 1e8 * static_cast<double>(i) / (N - 1));
        }
        CloudView cloud = b.view();
        cloud.data_size = cloud.point_count * cloud.point_step - 1;
        auto result = ingest(cloud, cfg);
        check(!result.time.ok, "short data is rejected");
        check(result.points.empty(), "short data yields no points");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("timestamp", 8);
        b.alloc(10);
        b.header = kHeader;
        for (size_t i = 0; i < 10; ++i) {
            b.xyz(i, 1.0, 2.0, 3.0);
            b.set(i, "timestamp", 1e7 * static_cast<double>(i));
        }
        CloudView cloud = b.view();
        cloud.width = 3;
        auto result = ingest(cloud, cfg);
        check(!result.time.ok, "width that does not divide point_count is rejected");
        check(result.points.empty(), "bad width yields no points");
    }
    {
        Builder b;
        b.add("x", 7);
        b.add("y", 7);
        b.add("z", 7);
        b.add("timestamp", 8);
        b.alloc(8);
        b.header = kHeader;
        for (size_t i = 0; i < 8; ++i) {
            b.xyz(i, 1.0, 2.0, 3.0);
            b.set(i, "timestamp", 1e7 * static_cast<double>(i));
        }
        CloudView cloud = b.view();
        cloud.width = 4;
        cloud.row_step = cloud.point_step * 4 - 1;
        cloud.data_size = cloud.point_count * cloud.point_step;
        auto result = ingest(cloud, cfg);
        check(!result.time.ok, "short row_step is rejected");
        check(result.points.empty(), "short row_step yields no points");
    }
    {
        // point_count * point_step wraps to 0, so a multiply against this
        // 128-byte buffer used to pass and then reserve() threw.
        static_assert(sizeof(size_t) >= 8, "wrap cases assume a 64-bit size_t");
        std::vector<uint8_t> bytes(128, 0);
        CloudView cloud;
        cloud.fields.push_back({"x", 0, 7, 1});
        cloud.fields.push_back({"y", 4, 7, 1});
        cloud.fields.push_back({"z", 8, 7, 1});
        cloud.data = bytes.data();
        cloud.data_size = bytes.size();
        cloud.point_count = size_t{1} << 60;
        cloud.width = size_t{1} << 30;
        cloud.point_step = 16;
        checkRejectedLayout(cloud, cfg, "wrapping packed count");
    }
    {
        // (height - 1) * row_step wraps to a span smaller than 128 bytes.
        std::vector<uint8_t> bytes(128, 0);
        CloudView cloud;
        cloud.fields.push_back({"x", 0, 7, 1});
        cloud.fields.push_back({"y", 4, 7, 1});
        cloud.fields.push_back({"z", 8, 7, 1});
        cloud.data = bytes.data();
        cloud.data_size = bytes.size();
        cloud.width = size_t{1} << 27;
        cloud.point_count = cloud.width * ((size_t{1} << 34) + 1);
        cloud.point_step = 16;
        cloud.row_step = uint32_t{1} << 31;
        checkRejectedLayout(cloud, cfg, "wrapping organized span");
    }

    std::printf(
        "\n%s (%d failure%s)\n",
        failures == 0 ? "ALL CHECKS PASSED" : "FAILURES",
        failures,
        failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
