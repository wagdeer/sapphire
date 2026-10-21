#include <algorithm>
#include <cmath>
#include <cstring>
#include <backend/registration/descriptor/feature_block.hpp>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace sapphire::backend::bow {
namespace {
void put_u32(std::vector<uint8_t> &data, uint32_t value) {
    for (int i = 0; i < 4; ++i)
        data.push_back(static_cast<uint8_t>(value >> (8 * i)));
}
void put_float(std::vector<uint8_t> &data, float value) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    uint32_t bits;
    std::memcpy(&bits, &value, 4);
    put_u32(data, bits);
}
struct Reader {
    const uint8_t *data;
    size_t left;
    uint32_t u32() {
        if (left < 4)
            throw std::runtime_error("Truncated BoW feature block");
        uint32_t value = 0;
        for (int i = 0; i < 4; ++i)
            value |= uint32_t(data[i]) << (8 * i);
        data += 4;
        left -= 4;
        return value;
    }
    float real() {
        uint32_t bits = u32();
        float value;
        std::memcpy(&value, &bits, 4);
        return value;
    }
};
std::shared_ptr<const FeatureBlock> create_unquantized_owned(std::vector<cv::KeyPoint>&& keypoints,
    std::vector<cv::Point3f>&& points, cv::Mat&& descriptors, uint64_t model) {
    auto block = std::make_shared<FeatureBlock>();
    block->keypoints = std::move(keypoints);
    block->points = points.empty() ? std::vector<cv::Point3f>(block->keypoints.size(), {NAN,NAN,NAN}) : std::move(points);
    block->descriptors = std::move(descriptors);
    block->descriptor_model_id = model;
    block->feature_ids.resize(block->keypoints.size());
    std::iota(block->feature_ids.begin(), block->feature_ids.end(), 0);
    block->validate();
    return block;
}
} // namespace

std::shared_ptr<const FeatureBlock> FeatureBlock::create(uint64_t vocabulary_hash, const std::vector<uint32_t> &node_ids,
                                                         const std::vector<int> &word_ids, const std::vector<cv::KeyPoint> &keypoints,
                                                         const std::vector<cv::Point3f> &points, const cv::Mat &descriptors) {
    const size_t count = keypoints.size();
    if (count > max_features || node_ids.size() != count || word_ids.size() != count || (!points.empty() && points.size() != count) ||
        descriptors.rows != static_cast<int>(count) || (count && (descriptors.type() != CV_8UC1 || descriptors.cols != 32)))
        throw std::invalid_argument("Invalid BoW feature arrays");
    auto block = std::make_shared<FeatureBlock>();
    block->vocabulary_hash = vocabulary_hash;
    block->feature_ids.resize(count);
    std::iota(block->feature_ids.begin(), block->feature_ids.end(), 0);
    std::stable_sort(block->feature_ids.begin(), block->feature_ids.end(),
                     [&](uint32_t a, uint32_t b) { return node_ids[a] < node_ids[b]; });
    block->descriptors.create(static_cast<int>(count), 32, CV_8UC1);
    block->word_ids.reserve(count);
    block->keypoints.reserve(count);
    block->points.reserve(count);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t id = block->feature_ids[i];
        if (block->groups.empty() || block->groups.back().node_id != node_ids[id])
            block->groups.push_back({node_ids[id], i, i});
        ++block->groups.back().end;
        block->word_ids.push_back(word_ids[id]);
        block->keypoints.push_back(keypoints[id]);
        block->points.push_back(points.empty() ? cv::Point3f(nan, nan, nan) : points[id]);
        std::memcpy(block->descriptors.ptr(i), descriptors.ptr(id), 32);
    }
    block->validate();
    return block;
}

std::shared_ptr<const FeatureBlock> FeatureBlock::create_raw(const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<cv::Point3f>& points, const cv::Mat& descriptors) {
    return create_raw_owned(std::vector<cv::KeyPoint>(keypoints), std::vector<cv::Point3f>(points), descriptors.clone());
}
std::shared_ptr<const FeatureBlock> FeatureBlock::create_raw_owned(std::vector<cv::KeyPoint>&& keypoints,
    std::vector<cv::Point3f>&& points, cv::Mat&& descriptors) {
    return create_unquantized_owned(std::move(keypoints), std::move(points), std::move(descriptors), 0);
}
std::shared_ptr<const FeatureBlock> FeatureBlock::create_learned(const std::vector<cv::KeyPoint>& keys,
    const std::vector<cv::Point3f>& xyz, const cv::Mat& values, uint64_t model) {
    return create_learned_owned(std::vector<cv::KeyPoint>(keys), std::vector<cv::Point3f>(xyz), values.empty() ? cv::Mat(values.rows, values.cols, values.type()) : values.clone(), model);
}
std::shared_ptr<const FeatureBlock> FeatureBlock::create_learned_owned(std::vector<cv::KeyPoint>&& keys,
    std::vector<cv::Point3f>&& xyz, cv::Mat&& values, uint64_t model) {
    if (!model) throw std::invalid_argument("Missing learned feature model identity");
    return create_unquantized_owned(std::move(keys), std::move(xyz), std::move(values), model);
}
uint32_t FeatureBlock::tree_node(size_t row) const {
    if (row >= keypoints.size()) throw std::out_of_range("Feature row");
    auto it = std::upper_bound(groups.begin(), groups.end(), row, [](size_t r, const FeatureGroup& g) { return r < g.end; });
    return it == groups.end() ? invalid_node_id : it->node_id;
}

void FeatureBlock::validate() const {
    const size_t count = keypoints.size();
    if (count > max_features || points.size() != count || (!is_raw() && word_ids.size() != count) || feature_ids.size() != count ||
        descriptors.rows != static_cast<int>(count) ||
        (count && (descriptors.type() != (descriptor_model_id ? CV_32FC1 : CV_8UC1) || (descriptor_model_id ? (descriptors.cols != 64 && descriptors.cols != 256) : descriptors.cols != 32) || !descriptors.isContinuous())))
        throw std::runtime_error("Invalid BoW feature block dimensions");
    if (descriptor_model_id) {
        if (descriptors.type() != CV_32FC1 || (descriptors.cols != 64 && descriptors.cols != 256)) throw std::runtime_error("Missing learned descriptor layout");
        if (!is_raw()) throw std::runtime_error("Learned features require bounded unquantized storage");
        for (size_t i=0;i<count;++i) {
            double norm=0;const auto* row=descriptors.ptr<float>(int(i));
            for(int j=0;j<descriptors.cols;++j) { if(!std::isfinite(row[j])) throw std::runtime_error("Nonfinite learned feature"); norm+=double(row[j])*row[j]; }
            if(std::abs(std::sqrt(norm)-1)>1e-3) throw std::runtime_error("Unnormalized learned feature");
        }
    }
    if (is_raw()) {
        if (!groups.empty()) throw std::runtime_error("Raw ORB cannot have vocabulary groups");
        for (uint32_t i=0; i<count; ++i) if (feature_ids[i] != i) throw std::runtime_error("Raw ORB feature order must be unchanged");
        return;
    }
    std::vector<uint8_t> seen(count, 0);
    uint32_t offset = 0, previous = 0;
    for (const auto &group : groups) {
        if (group.begin != offset || group.end <= group.begin || group.end > count || (offset && group.node_id <= previous))
            throw std::runtime_error("Invalid BoW group directory");
        for (uint32_t i = group.begin; i < group.end; ++i) {
            if (feature_ids[i] >= count || seen[feature_ids[i]]++ || (i > group.begin && feature_ids[i] <= feature_ids[i - 1]))
                throw std::runtime_error("Invalid BoW stable feature order");
        }
        offset = group.end;
        previous = group.node_id;
    }
    if (offset != count)
        throw std::runtime_error("Incomplete BoW group directory");
}

std::vector<uint8_t> FeatureBlock::encode() const {
    validate();
    std::vector<uint8_t> data;
    const size_t row_bytes = descriptor_model_id ? 44 + size_t(descriptors.cols) * 4 : is_raw() ? 76 : 80;
    data.reserve(28 + (descriptor_model_id ? (descriptors.cols == 64 ? 12 : 8) : 0) + groups.size() * 12 + keypoints.size() * row_bytes);
    put_u32(data, 0x574f4247); // GBOW, explicit little endian; never serialize C++ object storage.
    put_u32(data, descriptor_model_id ? (descriptors.cols == 64 ? 4 : 3) : is_raw() ? 2 : format_version);
    put_u32(data, vocabulary_hash);
    put_u32(data, vocabulary_hash >> 32);
    put_u32(data, is_raw() ? 0 : group_level);
    put_u32(data, keypoints.size());
    put_u32(data, groups.size());
    if(descriptor_model_id) { put_u32(data,descriptor_model_id); put_u32(data,descriptor_model_id>>32); }
    if(descriptor_model_id && descriptors.cols == 64) put_u32(data,64);
    for (const auto &group : groups) {
        put_u32(data, group.node_id);
        put_u32(data, group.begin);
        put_u32(data, group.end);
    }
    for (size_t i = 0; i < keypoints.size(); ++i) {
        put_u32(data, feature_ids[i]);
        if (!is_raw()) put_u32(data, static_cast<uint32_t>(word_ids[i]));
        const auto &keypoint = keypoints[i];
        put_float(data, keypoint.pt.x);
        put_float(data, keypoint.pt.y);
        put_float(data, keypoint.size);
        put_float(data, keypoint.angle);
        put_float(data, keypoint.response);
        put_u32(data, static_cast<uint32_t>(keypoint.octave));
        put_u32(data, static_cast<uint32_t>(keypoint.class_id));
        put_float(data, points[i].x);
        put_float(data, points[i].y);
        put_float(data, points[i].z);
    }
    if (descriptor_model_id) {
        for (int i = 0; i < descriptors.rows; ++i) {
            const float* row = descriptors.ptr<float>(i);
            for (int c = 0; c < descriptors.cols; ++c) put_float(data, row[c]);
        }
    } else if (!descriptors.empty())
        data.insert(data.end(), descriptors.data, descriptors.data + descriptors.total());
    return data;
}

std::shared_ptr<const FeatureBlock> FeatureBlock::decode(const void *data, size_t size) {
    if (!data || size < 28)
        throw std::runtime_error("Missing BoW feature block header");
    Reader reader{static_cast<const uint8_t *>(data), size};
    if (reader.u32() != 0x574f4247) throw std::runtime_error("Unsupported feature block magic");
    const auto version = reader.u32();
    if (version != 1 && version != 2 && version != 3 && version != 4) throw std::runtime_error("Unsupported feature block format");
    const bool raw = version >= 2;
    auto block = std::make_shared<FeatureBlock>();
    block->vocabulary_hash = reader.u32();
    block->vocabulary_hash |= uint64_t(reader.u32()) << 32;
    if (reader.u32() != (raw ? 0 : group_level) || (raw && block->vocabulary_hash))
        throw std::runtime_error("Unsupported BoW group level");
    const uint32_t count = reader.u32(), group_count = reader.u32();
    if(version>=3) {
        block->descriptor_model_id=reader.u32(); block->descriptor_model_id|=uint64_t(reader.u32())<<32;
        if(!block->descriptor_model_id) throw std::runtime_error("Invalid learned feature domain/capacity");
    }
    const int dimensions = version == 4 ? int(reader.u32()) : version == 3 ? 256 : 32;
    if (version == 4 && dimensions != 64) throw std::runtime_error("Invalid learned feature dimensions");
    if (count > max_features || group_count > count || (raw && group_count) || reader.left != size_t(group_count) * 12 + size_t(count) * (version>=3 ? 44 + size_t(dimensions) * 4 : raw ? 76 : 80))
        throw std::runtime_error("Invalid BoW feature block lengths");
    block->groups.reserve(group_count);
    block->feature_ids.reserve(count);
    block->keypoints.reserve(count);
    block->points.reserve(count);
    if (!raw) block->word_ids.reserve(count);
    for (uint32_t i = 0; i < group_count; ++i) {
        FeatureGroup group;
        group.node_id = reader.u32();
        group.begin = reader.u32();
        group.end = reader.u32();
        block->groups.push_back(group);
    }
    for (uint32_t i = 0; i < count; ++i) {
        block->feature_ids.push_back(reader.u32());
        if (!raw) block->word_ids.push_back(static_cast<int32_t>(reader.u32()));
        cv::KeyPoint keypoint;
        keypoint.pt.x = reader.real();
        keypoint.pt.y = reader.real();
        keypoint.size = reader.real();
        keypoint.angle = reader.real();
        keypoint.response = reader.real();
        keypoint.octave = static_cast<int32_t>(reader.u32());
        keypoint.class_id = static_cast<int32_t>(reader.u32());
        block->keypoints.push_back(keypoint);
        cv::Point3f point;
        point.x = reader.real();
        point.y = reader.real();
        point.z = reader.real();
        block->points.push_back(point);
    }
    block->descriptors.create(count, dimensions, version>=3 ? CV_32FC1 : CV_8UC1);
    if(version>=3) {
        for (uint32_t i = 0; i < count; ++i) {
            float* row = block->descriptors.ptr<float>(i);
            for (int c = 0; c < dimensions; ++c) row[c] = reader.real();
        }
    } else if (count)
        std::memcpy(block->descriptors.data, reader.data, size_t(count) * 32);
    block->validate();
    return block;
}

size_t FeatureBlock::memory_size() const {
    return sizeof(*this) + groups.capacity() * sizeof(FeatureGroup) + feature_ids.capacity() * 4 + word_ids.capacity() * sizeof(int) +
           keypoints.capacity() * sizeof(cv::KeyPoint) + points.capacity() * sizeof(cv::Point3f) + descriptors.total()*descriptors.elemSize();
}

} // namespace sapphire::backend::bow
