#include "database/compression.hpp"

#include <zlib.h>
#include <Eigen/Cholesky>
#include <type_traits>

#include <tools/logging.hpp>
#include <cstdint>
#include <cstring>
#include <limits>
#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace sapphire {

namespace {
constexpr int kSerializedDepthMask = 7;
constexpr int kSerializedCnShift = 3;
int serializeMatType(int type) {
  const int depth = CV_MAT_DEPTH(type);
  EXT_ASSERT_MSG(depth <= kSerializedDepthMask, "{}",
                 fmt::format("Cannot serialize cv::Mat depth {} (type {}): the database "
                             "format only supports depths 0-{}.",
                             depth, type, kSerializedDepthMask));
  return depth + ((CV_MAT_CN(type) - 1) << kSerializedCnShift);
}

int deserializeMatType(int serializedType) {
  return CV_MAKETYPE(serializedType & kSerializedDepthMask, ((serializedType >> kSerializedCnShift) & 511) + 1);
}

cv::Mat decodeData(const unsigned char *bytes, unsigned long size) {
  cv::Mat data;
  if (bytes && size >= 3 * sizeof(int)) {
    int height = 0;
    int width = 0;
    int serializedType = 0;
    std::memcpy(&height, &bytes[size - 3 * sizeof(int)], sizeof(height));
    std::memcpy(&width, &bytes[size - 2 * sizeof(int)], sizeof(width));
    std::memcpy(&serializedType, &bytes[size - sizeof(int)], sizeof(serializedType));
    const int type = deserializeMatType(serializedType);

    data = cv::Mat(height, width, type);
    uLongf totalUncompressed = uLongf(data.total()) * uLongf(data.elemSize());
    const int errCode = uncompress(reinterpret_cast<Bytef *>(data.data), &totalUncompressed, reinterpret_cast<const Bytef *>(bytes), uLong(size));

    if (errCode == Z_MEM_ERROR) {
      EXT_LOG_ERROR("Z_MEM_ERROR : Insufficient memory.");
    } else if (errCode == Z_BUF_ERROR) {
      EXT_LOG_ERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
    } else if (errCode == Z_DATA_ERROR) {
      EXT_LOG_ERROR("Z_DATA_ERROR : The compressed data (referenced by source) was corrupted.");
    }
  }
  return data;
}

}  // namespace

cv::Mat compressData(const cv::Mat &data) {
  cv::Mat bytes;
  if (!data.empty()) {
    uLong sourceLen = uLong(data.total()) * uLong(data.elemSize());
    uLong destLen = compressBound(sourceLen);
    if (destLen > std::numeric_limits<int>::max() - 3 * sizeof(int))
      throw std::length_error("Compressed matrix exceeds archive representation");
    bytes = cv::Mat(1, destLen + 3 * sizeof(int), CV_8UC1);
    int errCode = compress((Bytef *)bytes.data, &destLen, (const Bytef *)data.data, sourceLen);
    bytes = cv::Mat(bytes, cv::Rect(0, 0, destLen + 3 * sizeof(int), 1));
    const int serializedType = serializeMatType(data.type());
    std::memcpy(&bytes.data[destLen], &data.rows, sizeof(data.rows));
    std::memcpy(&bytes.data[destLen + sizeof(int)], &data.cols, sizeof(data.cols));
    std::memcpy(&bytes.data[destLen + 2 * sizeof(int)], &serializedType, sizeof(serializedType));

    if (errCode == Z_MEM_ERROR) {
      EXT_LOG_ERROR("Z_MEM_ERROR : Insufficient memory.");
    } else if (errCode == Z_BUF_ERROR) {
      EXT_LOG_ERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
    }
  }
  // A matrix ROI retains compressBound() bytes. Archives own only the payload.
  return bytes.clone();
}

cv::Mat uncompressData(const cv::Mat &bytes) {
  EXT_ASSERT(bytes.empty() || bytes.type() == CV_8UC1);
  return decodeData(bytes.data, bytes.total());
}

cv::Mat compressString(const std::string &str) {
  // +1 to include null character
  return compressData(cv::Mat(1, str.size() + 1, CV_8SC1, (void *)str.data()));
}

std::string uncompressString(const cv::Mat &bytes) {
  cv::Mat strMat = uncompressData(bytes);
  if (!strMat.empty()) {
    EXT_ASSERT(strMat.type() == CV_8SC1 && strMat.rows == 1);
    return (const char *)strMat.data;
  }
  return "";
}

} /* namespace sapphire */

namespace sapphire::lidar {
namespace {
constexpr char magic[] = "SAPLID01", supported_magic[] = "SAPLID02";
constexpr std::size_t record_bytes = 85, header_bytes = 16, prefix_bytes = 24, support_bytes = 33 * sizeof(double);
template <class T>
void put(unsigned char*& out, T value) {
  static_assert(sizeof(T) == 4 || sizeof(T) == 8);
  using Bits = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
  Bits bits;
  std::memcpy(&bits, &value, sizeof(T));
  for (size_t i = 0; i < sizeof(T); ++i) *out++ = static_cast<unsigned char>(bits >> (8 * i));
}
template <class T>
T get(const unsigned char*& in) {
  using Bits = std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>;
  Bits bits = 0;
  for (size_t i = 0; i < sizeof(T); ++i) bits |= Bits(*in++) << (8 * i);
  T value;
  std::memcpy(&value, &bits, sizeof(T));
  return value;
}
}  // namespace
namespace {
bool has_support_prefix(const void* prefix, std::size_t size) {
  return prefix && size >= 8 && !std::memcmp(prefix, supported_magic, 8);
}
std::size_t payload_size(const GaussianCloudSize& size, bool supported) {
  size.bytes();
  const auto limit = std::size_t(std::numeric_limits<int>::max());
  const auto header = supported ? 0 : header_bytes;
  if (size.points > (limit - header) / record_bytes ||
      size.supports > (limit - header - size.points * record_bytes) / support_bytes)
    throw std::length_error("Gaussian archive exceeds matrix representation");
  return header + size.points * record_bytes + size.supports * support_bytes;
}
void put_support(unsigned char*& out, const GaussianSupport& s) {
  for (const auto* matrix : {&s.covariance_lower, &s.plane_upper, &s.box_axes})
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) put(out, (*matrix)(r, c));
  for (const auto* vector : {&s.box_center, &s.box_half})
    for (int i = 0; i < 3; ++i) put(out, (*vector)[i]);
}
std::shared_ptr<const GaussianSupport> get_support(const unsigned char*& in) {
  auto s = std::make_shared<GaussianSupport>();
  for (auto* matrix : {&s->covariance_lower, &s->plane_upper, &s->box_axes})
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) (*matrix)(r, c) = get<double>(in);
  for (auto* vector : {&s->box_center, &s->box_half})
    for (int i = 0; i < 3; ++i) (*vector)[i] = get<double>(in);
  return s;
}
} // namespace
cv::Mat encode(const GaussianCloud& cloud) {
  validateGaussianCloud(cloud);
  const auto size = gaussian_cloud_size(cloud);
  const bool supported = size.supports != 0;
  cv::Mat raw(1, int(payload_size(size, supported)), CV_8UC1);
  auto* out = raw.ptr();
  if (!supported) {
    std::memcpy(out, magic, 8); out += 8; put(out, uint64_t(size.points));
  }
  for (const auto& p : cloud) {
    put(out, p.voxel_key.x); put(out, p.voxel_key.y); put(out, p.voxel_key.z);
    put(out, int32_t(p.voxel_key.level)); put(out, int32_t(p.N));
    for (int i = 0; i < 3; ++i) put(out, p.mean[i]);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) put(out, p.covariance(r, c));
    put(out, p.radius);
    *out++ = (p.is_plane ? 1 : 0) | (p.support ? 2 : 0);
    if (p.support) put_support(out, *p.support);
  }
  auto compressed = compressData(raw);
  if (!supported) return compressed;
  if (compressed.total() > std::size_t(std::numeric_limits<int>::max()) - prefix_bytes)
    throw std::length_error("Gaussian archive exceeds matrix representation");
  cv::Mat archive(1, int(prefix_bytes + compressed.total()), CV_8UC1);
  out = archive.ptr(); std::memcpy(out, supported_magic, 8); out += 8;
  put(out, uint64_t(size.points)); put(out, uint64_t(size.supports));
  std::memcpy(out, compressed.ptr(), compressed.total());
  return archive;
}
GaussianCloudSize decoded_size(const void* prefix, std::size_t prefix_size,
                              const void* trailer, std::size_t trailer_size) {
  if (!trailer || trailer_size != 12) throw std::runtime_error("Invalid LiDAR geometry matrix trailer");
  int shape[3]; std::memcpy(shape, trailer, 12);
  if (shape[0] != 1 || shape[1] <= 0 || shape[2] != CV_8UC1)
    throw std::runtime_error("Invalid LiDAR geometry matrix shape");
  GaussianCloudSize size;
  if (has_support_prefix(prefix, prefix_size)) {
    if (prefix_size < prefix_bytes) throw std::runtime_error("Truncated Gaussian support header");
    const auto* in = static_cast<const unsigned char*>(prefix) + 8;
    const auto points = get<uint64_t>(in), supports = get<uint64_t>(in);
    if (!points || !supports || supports > points || points > std::numeric_limits<std::size_t>::max())
      throw std::runtime_error("Invalid Gaussian support counts");
    size = {std::size_t(points), std::size_t(supports)};
    if (payload_size(size, true) != std::size_t(shape[1])) throw std::runtime_error("Gaussian support payload size mismatch");
  } else {
    if (shape[1] <= int(header_bytes) || (shape[1] - header_bytes) % record_bytes)
      throw std::runtime_error("Invalid LiDAR geometry matrix shape");
    size.points = (shape[1] - header_bytes) / record_bytes;
  }
  size.bytes(); return size;
}
GaussianCloudPtr decode(const cv::Mat& archive) {
  if (archive.empty()) return {};
  if (archive.type() != CV_8UC1 || !archive.isContinuous() || archive.total() < 12)
    throw std::runtime_error("Invalid LiDAR archive");
  int shape[3]; std::memcpy(shape, archive.ptr() + archive.total() - 12, 12);
  const auto size = decoded_size(archive.ptr(), std::min(prefix_bytes, archive.total()), shape, sizeof(shape));
  const bool supported = has_support_prefix(archive.ptr(), archive.total());
  const auto offset = supported ? prefix_bytes : 0;
  if (archive.total() < offset + 12 || archive.total() - offset - 12 > std::numeric_limits<uInt>::max())
    throw std::runtime_error("Invalid LiDAR archive length");
  struct Inflater {
    z_stream stream{};
    Inflater() { if (inflateInit(&stream) != Z_OK) throw std::runtime_error("Cannot initialize geometry decompression"); }
    ~Inflater() { inflateEnd(&stream); }
  } inflater;
  auto& stream = inflater.stream;
  stream.next_in = archive.data + offset;
  stream.avail_in = static_cast<uInt>(archive.total() - offset - 12);
  std::array<unsigned char, 64 * 1024> chunk;
  std::vector<unsigned char> raw;
  int status;
  do {
    stream.next_out = chunk.data(); stream.avail_out = chunk.size();
    status = inflate(&stream, Z_NO_FLUSH);
    const auto produced = chunk.size() - stream.avail_out;
    if ((status != Z_OK && status != Z_STREAM_END) || raw.size() + produced > std::size_t(shape[1]))
      throw std::runtime_error("Corrupt LiDAR geometry compression");
    raw.insert(raw.end(), chunk.data(), chunk.data() + produced);
  } while (status != Z_STREAM_END);
  if (raw.size() != std::size_t(shape[1]) || stream.avail_in) throw std::runtime_error("Invalid LiDAR geometry length");
  const auto* in = raw.data(); const auto* end = raw.data() + raw.size();
  if (!supported) {
    if (raw.size() < header_bytes || std::memcmp(in, magic, 8)) throw std::runtime_error("Invalid LiDAR geometry header");
    in += 8;
    if (get<uint64_t>(in) != size.points) throw std::runtime_error("Invalid LiDAR geometry count");
  }
  auto cloud = std::make_shared<GaussianCloud>(size.points);
  std::size_t support_count = 0;
  for (auto& p : *cloud) {
    if (std::size_t(end - in) < record_bytes) throw std::runtime_error("Truncated Gaussian record");
    p.voxel_key.x = get<int64_t>(in); p.voxel_key.y = get<int64_t>(in); p.voxel_key.z = get<int64_t>(in);
    p.voxel_key.level = get<int32_t>(in); p.N = get<int32_t>(in);
    for (int i = 0; i < 3; ++i) p.mean[i] = get<float>(in);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) p.covariance(r, c) = get<float>(in);
    p.radius = get<float>(in); const auto flags = *in++;
    if (flags > (supported ? 3 : 1)) throw std::runtime_error("Invalid LiDAR geometry flags");
    p.is_plane = flags & 1;
    if (flags & 2) {
      if (++support_count > size.supports || std::size_t(end - in) < support_bytes)
        throw std::runtime_error("Truncated Gaussian support");
      p.support = get_support(in);
    }
  }
  if (in != end || support_count != size.supports) throw std::runtime_error("Gaussian support count mismatch");
  validateGaussianCloud(*cloud); return cloud;
}
} // namespace sapphire::lidar
