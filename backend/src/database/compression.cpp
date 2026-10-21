#include "backend/database/compression.hpp"

#include <zlib.h>

#include <cstdint>
#include <cstring>
#include <backend/tools/logging.hpp>
#include <limits>
#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace sapphire::backend {

namespace {
class RvlCodec {
 public:
  RvlCodec();
  // Compress input data into output. The size of output can be equal to (1.5 * numPixels + 4) in the worst case.
  int CompressRVL(const uint16_t *input, unsigned char *output, int numPixels);
  // Decompress input data into output. The size of output must be equal to numPixels.
  void DecompressRVL(const unsigned char *input, uint16_t *output, int numPixels);

 private:
  RvlCodec(const RvlCodec &);
  RvlCodec &operator=(const RvlCodec &);

  void EncodeVLE(int value);
  int DecodeVLE();

  int *buffer_;
  int *pBuffer_;
  int word_;
  int nibblesWritten_;
};

RvlCodec::RvlCodec() : buffer_(nullptr), pBuffer_(nullptr), word_(0), nibblesWritten_(0) {}

void RvlCodec::EncodeVLE(int value) {
  do {
    int nibble = value & 0x7;        // lower 3 bits
    if (value >>= 3) nibble |= 0x8;  // more to come
    word_ <<= 4;
    word_ |= nibble;
    if (++nibblesWritten_ == 8)  // output word
    {
      *pBuffer_++ = word_;
      nibblesWritten_ = 0;
      word_ = 0;
    }
  } while (value);
}

int RvlCodec::DecodeVLE() {
  unsigned int nibble;
  int value = 0, bits = 29;
  do {
    if (!nibblesWritten_) {
      word_ = *pBuffer_++;  // load word
      nibblesWritten_ = 8;
    }
    nibble = word_ & 0xf0000000;
    value |= (nibble << 1) >> bits;
    word_ <<= 4;
    nibblesWritten_--;
    bits -= 3;
  } while (nibble & 0x80000000);
  return value;
}

int RvlCodec::CompressRVL(const uint16_t *input, unsigned char *output, int numPixels) {
  buffer_ = pBuffer_ = reinterpret_cast<int *>(output);
  nibblesWritten_ = 0;
  const uint16_t *end = input + numPixels;
  uint16_t previous = 0;
  while (input != end) {
    int zeros = 0, nonzeros = 0;
    for (; (input != end) && !*input; input++, zeros++) {
    }
    EncodeVLE(zeros);  // number of zeros
    for (const uint16_t *p = input; (p != end) && *p++; nonzeros++) {
    }
    EncodeVLE(nonzeros);  // number of nonzeros
    for (int i = 0; i < nonzeros; i++) {
      uint16_t current = *input++;
      int delta = current - previous;
      int positive = (delta << 1) ^ (delta >> 31);
      EncodeVLE(positive);  // nonzero value
      previous = current;
    }
  }
  if (nibblesWritten_)  // last few values
    *pBuffer_++ = word_ << 4 * (8 - nibblesWritten_);
  return static_cast<int>((unsigned char *)pBuffer_ - (unsigned char *)buffer_);  // num bytes
}

void RvlCodec::DecompressRVL(const unsigned char *input, uint16_t *output, int numPixels) {
  buffer_ = pBuffer_ = const_cast<int *>(reinterpret_cast<const int *>(input));
  nibblesWritten_ = 0;
  uint16_t current, previous = 0;
  int numPixelsToDecode = numPixels;
  while (numPixelsToDecode) {
    int zeros = DecodeVLE();  // number of zeros
    numPixelsToDecode -= zeros;
    for (; zeros; zeros--) *output++ = 0;
    int nonzeros = DecodeVLE();  // number of nonzeros
    numPixelsToDecode -= nonzeros;
    for (; nonzeros; nonzeros--) {
      int positive = DecodeVLE();  // nonzero value
      int delta = (positive >> 1) ^ -(positive & 1);
      current = previous + delta;
      *output++ = current;
      previous = current;
    }
  }
}

const int kSerializedCnShift = 3;
const int kSerializedDepthMask = (1 << kSerializedCnShift) - 1;
constexpr unsigned char kRvlNode[] = {'D', 'E', 'P', 'T', 'H', 'R', 'V', 'L'};
constexpr size_t kRvlHeaderSize = 16;

int serializeMatType(int type) {
  const int depth = CV_MAT_DEPTH(type);
  EXT_ASSERT_MSG(depth <= kSerializedDepthMask, "{}",
                 fmt::format("Cannot serialize cv::Mat depth {} (type {}): the database "
                             "format only supports depths 0-{}.",
                             depth, type, kSerializedDepthMask));
  return depth + ((CV_MAT_CN(type) - 1) << kSerializedCnShift);
}

int deserializeMatType(int serializedType) { return CV_MAKETYPE(serializedType & kSerializedDepthMask, ((serializedType >> kSerializedCnShift) & 511) + 1); }

bool hasValidDepthRvlHeader(const unsigned char *bytes, size_t size) {
  if (bytes == nullptr || size < kRvlHeaderSize || std::memcmp(bytes, kRvlNode, sizeof(kRvlNode)) != 0) {
    return false;
  }

  uint32_t cols = 0;
  uint32_t rows = 0;
  std::memcpy(&cols, bytes + 8, sizeof(cols));
  std::memcpy(&rows, bytes + 12, sizeof(rows));
  return cols > 0 && rows > 0 && uint64_t(cols) * uint64_t(rows) <= uint64_t(std::numeric_limits<int>::max());
}

cv::Mat decodeDepthRvl(const unsigned char *bytes, size_t size) {
  EXT_ASSERT_MSG(hasValidDepthRvlHeader(bytes, size), "Depth buffer has an invalid or truncated DEPTHRVL header.");

  uint32_t cols = 0;
  uint32_t rows = 0;
  std::memcpy(&cols, bytes + 8, sizeof(cols));
  std::memcpy(&rows, bytes + 12, sizeof(rows));

  cv::Mat depth(int(rows), int(cols), CV_16UC1);
  RvlCodec rvl;
  rvl.DecompressRVL(bytes + kRvlHeaderSize, depth.ptr<uint16_t>(), int(cols * rows));
  return depth;
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

cv::Mat compressImage(const cv::Mat &image) {
  std::vector<unsigned char> bytes;
  if (!image.empty()) {
    EXT_ASSERT_MSG(image.depth() == CV_8U, "Image compression accepts only 8-bit images; depth must use compressDepth().");
    cv::imencode(".jpg", image, bytes);
  }
  if (!bytes.empty()) {
    return cv::Mat(1, (int)bytes.size(), CV_8UC1, bytes.data()).clone();
  }
  return cv::Mat();
}

cv::Mat uncompressImage(const cv::Mat &bytes) {
  if (bytes.empty()) {
    return cv::Mat();
  }
  EXT_ASSERT(bytes.type() == CV_8UC1);
  return cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
}

cv::Mat compressDepth(const cv::Mat &depth) {
  if (!depth.empty()) {
    EXT_ASSERT_MSG(depth.type() == CV_16UC1, "Depth compression accepts only CV_16UC1 millimeter depth.");
    EXT_ASSERT_MSG(depth.isContinuous(), "Depth compression requires a continuous matrix.");

    const int numPixels = depth.rows * depth.cols;
    cv::Mat bytes(1, 3 * numPixels + 20, CV_8UC1);
    std::memcpy(bytes.data, kRvlNode, sizeof(kRvlNode));
    const uint32_t cols = uint32_t(depth.cols);
    const uint32_t rows = uint32_t(depth.rows);
    std::memcpy(bytes.data + 8, &cols, sizeof(cols));
    std::memcpy(bytes.data + 12, &rows, sizeof(rows));

    RvlCodec rvl;
    const int compressedSize = rvl.CompressRVL(depth.ptr<uint16_t>(), bytes.data + kRvlHeaderSize, numPixels);
    return bytes.colRange(0, int(kRvlHeaderSize) + compressedSize).clone();
  }
  return cv::Mat();
}

cv::Mat uncompressDepth(const cv::Mat &bytes) {
  if (bytes.empty()) {
    return cv::Mat();
  }
  EXT_ASSERT_MSG(bytes.type() == CV_8UC1 && bytes.isContinuous(), "Compressed RVL depth must be a continuous CV_8UC1 buffer.");
  return decodeDepthRvl(bytes.data, bytes.total());
}

bool isDepthRvl(const cv::Mat &bytes) {
  return !bytes.empty() && bytes.type() == CV_8UC1 && bytes.isContinuous() && hasValidDepthRvlHeader(bytes.data, bytes.total());
}

cv::Mat compressData(const cv::Mat &data) {
  cv::Mat bytes;
  if (!data.empty()) {
    uLong sourceLen = uLong(data.total()) * uLong(data.elemSize());
    uLong destLen = compressBound(sourceLen);
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

cv::Mat compressGridCells(const GridCells &cells) {
  if (cells.empty()) {
    return {};
  }
  if (!cells.valid() || cells.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("Invalid local grid cells for compression");
  }
  const int channels = cells.hasNormals() ? 6 : (cells.is3D() ? 3 : 2);
  simd::aligned_vector<float> interleaved(cells.size() * static_cast<std::size_t>(channels));
  for (std::size_t i = 0; i < cells.size(); ++i) {
    float *value = interleaved.data() + i * static_cast<std::size_t>(channels);
    value[0] = cells.xy.x[i];
    value[1] = cells.xy.y[i];
    if (channels >= 3) {
      value[2] = cells.z[i];
    }
    if (channels == 6) {
      value[3] = cells.normals.x[i];
      value[4] = cells.normals.y[i];
      value[5] = cells.normals.z[i];
    }
  }

  const uLong sourceLen = static_cast<uLong>(interleaved.size() * sizeof(float));
  uLong destinationLength = compressBound(sourceLen);
  cv::Mat bytes(1, static_cast<int>(destinationLength + 3U * sizeof(int)), CV_8UC1);
  const int error = compress(bytes.data, &destinationLength, reinterpret_cast<const Bytef *>(interleaved.data()), sourceLen);
  if (error != Z_OK) {
    throw std::runtime_error("Failed to compress local grid cells");
  }
  bytes = bytes.colRange(0, static_cast<int>(destinationLength + 3U * sizeof(int)));
  const int rows = 1;
  const int columns = static_cast<int>(cells.size());
  const int serializedType = serializeMatType(CV_MAKETYPE(CV_32F, channels));
  std::memcpy(bytes.data + destinationLength, &rows, sizeof(rows));
  std::memcpy(bytes.data + destinationLength + sizeof(int), &columns, sizeof(columns));
  std::memcpy(bytes.data + destinationLength + 2U * sizeof(int), &serializedType, sizeof(serializedType));
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

} /* namespace sapphire::backend */
