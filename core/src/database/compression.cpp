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
constexpr char magic[] = "SAPLID01";
constexpr std::size_t record_bytes = 85, header_bytes = 16;
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
cv::Mat encode(const GaussianCloud& cloud) {
  validateGaussianCloud(cloud);
  if (cloud.size() > (std::numeric_limits<int>::max() - header_bytes) / record_bytes)
    throw std::length_error("Gaussian archive exceeds matrix representation");
  cv::Mat raw(1, int(header_bytes + record_bytes * cloud.size()), CV_8UC1);
  auto* out = raw.ptr();
  std::memcpy(out, magic, 8);
  out += 8;
  put(out, uint64_t(cloud.size()));
  for (const auto& p : cloud) {
    put(out, p.voxel_key.x);
    put(out, p.voxel_key.y);
    put(out, p.voxel_key.z);
    put(out, int32_t(p.voxel_key.level));
    put(out, int32_t(p.N));
    for (int i = 0; i < 3; ++i) put(out, p.mean[i]);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) put(out, p.covariance(r, c));
    put(out, p.radius);
    *out++ = p.is_plane ? 1 : 0;
  }
  return compressData(raw);
}
GaussianCloudPtr decode(const cv::Mat& archive) {
  if (archive.empty()) return {};
  // Check the matrix trailer before allocating decompression output.
  if (archive.type() != CV_8UC1 || !archive.isContinuous() || archive.total() < 12) throw std::runtime_error("Invalid LiDAR archive");
  int shape[3];
  std::memcpy(shape, archive.ptr() + archive.total() - 12, 12);
  if (shape[0] != 1 || shape[1] <= int(header_bytes) || shape[2] != CV_8UC1 ||
      (shape[1] - header_bytes) % record_bytes || archive.total() - 12 > std::numeric_limits<uInt>::max())
    throw std::runtime_error("Invalid LiDAR geometry matrix shape");
  // The trailer is untrusted: grow only with actual decompressed output, never
  // allocate its advertised size. A chunk is a scratch buffer, not a cloud limit.
  struct Inflater {
    z_stream stream{};
    Inflater() {
      if (inflateInit(&stream) != Z_OK) throw std::runtime_error("Cannot initialize geometry decompression");
    }
    ~Inflater() { inflateEnd(&stream); }
  } inflater;
  auto& stream = inflater.stream;
  stream.next_in = archive.data;
  stream.avail_in = static_cast<uInt>(archive.total() - 12);
  std::array<unsigned char, 64 * 1024> chunk;
  std::vector<unsigned char> raw;
  bool header_checked = false;
  int status;
  do {
    stream.next_out = chunk.data();
    stream.avail_out = chunk.size();
    status = inflate(&stream, Z_NO_FLUSH);
    const auto produced = chunk.size() - stream.avail_out;
    if ((status != Z_OK && status != Z_STREAM_END) || raw.size() + produced > static_cast<std::size_t>(shape[1]))
      throw std::runtime_error("Corrupt LiDAR geometry compression");
    raw.insert(raw.end(), chunk.data(), chunk.data() + produced);
    if (!header_checked && raw.size() >= header_bytes) {
      if (std::memcmp(raw.data(), magic, 8)) throw std::runtime_error("Invalid LiDAR geometry header");
      const auto* header = raw.data() + 8;
      if (get<uint64_t>(header) != (shape[1] - header_bytes) / record_bytes)
        throw std::runtime_error("Invalid LiDAR geometry count");
      header_checked = true;
    }
  } while (status != Z_STREAM_END);
  if (!header_checked || raw.size() != static_cast<std::size_t>(shape[1]) || stream.avail_in)
    throw std::runtime_error("Invalid LiDAR geometry length");
  const auto* in = raw.data() + 8;
  const auto count = get<uint64_t>(in);
  auto cloud = std::make_shared<GaussianCloud>(size_t(count));
  for (auto& p : *cloud) {
    p.voxel_key.x = get<int64_t>(in);
    p.voxel_key.y = get<int64_t>(in);
    p.voxel_key.z = get<int64_t>(in);
    p.voxel_key.level = get<int32_t>(in);
    p.N = get<int32_t>(in);
    for (int i = 0; i < 3; ++i) p.mean[i] = get<float>(in);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) p.covariance(r, c) = get<float>(in);
    p.radius = get<float>(in);
    const auto plane = *in++;
    if (plane > 1) throw std::runtime_error("Invalid LiDAR plane flag");
    p.is_plane = plane;
  }
  validateGaussianCloud(*cloud);
  return cloud;
}
}  // namespace sapphire::lidar
