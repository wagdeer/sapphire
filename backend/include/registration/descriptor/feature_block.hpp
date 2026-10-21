#pragma once

#include <backend/common/visual_frame.hpp>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace sapphire::backend::bow {

struct WordCount {
    int word_id;
    uint32_t count;
};

struct FeatureGroup {
    uint32_t node_id;
    uint32_t begin;
    uint32_t end;
};

// Construct once, then share as const. Quantized arrays use stable group order;
// raw ORB arrays retain input row order without a vocabulary directory.
struct FeatureBlock : VisualFeatures {
    static constexpr uint32_t format_version = 1;
    static constexpr uint32_t max_features = 8192;
    static constexpr uint32_t group_level = 4;
    static constexpr uint32_t invalid_node_id = UINT32_MAX;
    uint64_t vocabulary_hash = 0;
    std::vector<FeatureGroup> groups;
    std::vector<uint32_t> feature_ids;
    std::vector<int> word_ids;

    static std::shared_ptr<const FeatureBlock> create(uint64_t vocabulary_hash, const std::vector<uint32_t> &node_ids,
                                                      const std::vector<int> &word_ids, const std::vector<cv::KeyPoint> &keypoints,
                                                      const std::vector<cv::Point3f> &points, const cv::Mat &descriptors);
    // Raw ORB has no vocabulary, visual words or tree groups. Serialized as v2.
    bool is_raw() const { return vocabulary_hash == 0 && word_ids.empty(); }
    static std::shared_ptr<const FeatureBlock> create_raw(const std::vector<cv::KeyPoint>& keypoints,
        const std::vector<cv::Point3f>& points, const cv::Mat& descriptors);
    // Transfer owned, continuous arrays into an immutable block. The caller must
    // retain no writable aliases; borrowed input must use create_raw() instead.
    static std::shared_ptr<const FeatureBlock> create_raw_owned(std::vector<cv::KeyPoint>&& keypoints,
        std::vector<cv::Point3f>&& points, cv::Mat&& descriptors);
    static std::shared_ptr<const FeatureBlock> create_learned(const std::vector<cv::KeyPoint>& keypoints,
        const std::vector<cv::Point3f>& points, const cv::Mat& descriptors, uint64_t model_id);
    // Same ownership contract as create_raw_owned(); no clone or conversion.
    static std::shared_ptr<const FeatureBlock> create_learned_owned(std::vector<cv::KeyPoint>&& keypoints,
        std::vector<cv::Point3f>&& points, cv::Mat&& descriptors, uint64_t model_id);
    uint32_t tree_node(size_t row) const;
    int word_id(size_t row) const { return word_ids.empty() ? -1 : word_ids.at(row); }
    void validate() const;
    std::vector<uint8_t> encode() const;
    static std::shared_ptr<const FeatureBlock> decode(const void *data, size_t size);
    size_t memory_size() const;
};

} // namespace sapphire::backend::bow
