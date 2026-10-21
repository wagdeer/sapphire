#pragma once
#include <cstring>
#include <vector>

#include "distance.hpp"

namespace sapphire::backend::feature {
// On-disk payload alone. The enclosing archive checks version, format, model,
// dimensions and checksum before selecting this compile-time codec.
template <class Format>
struct descriptor_codec {
  using traits = descriptor_traits<Format>;
  static constexpr size_t bytes = sizeof(descriptor_storage<Format>);
  static void append(std::vector<uint8_t>& out, descriptor_row<Format> row) {
    if constexpr (std::is_same_v<Format, Binary_U8>)
      out.insert(out.end(), row.data(), row.data() + traits::dimensions);
    else
      for (int i = 0; i < traits::dimensions; ++i) {
        if constexpr (std::is_same_v<typename traits::scalar, float>) {
          uint32_t bits;
          std::memcpy(&bits, row.data() + i, 4);
          for (int byte = 0; byte < 4; ++byte) out.push_back(uint8_t(bits >> (8 * byte)));
        } else {
          const uint16_t bits = row.data()[i].bits();
          out.push_back(uint8_t(bits));
          out.push_back(uint8_t(bits >> 8));
        }
      }
  }
  static descriptor_storage<Format> read(const uint8_t*& cursor, size_t& remaining) {
    if (remaining < bytes) throw std::runtime_error("Truncated typed descriptor payload");
    descriptor_storage<Format> result;
    if constexpr (std::is_same_v<Format, Binary_U8>)
      std::memcpy(result.data(), cursor, bytes);
    else
      for (int i = 0; i < traits::dimensions; ++i) {
        if constexpr (std::is_same_v<typename traits::scalar, float>) {
          uint32_t bits = 0;
          for (int byte = 0; byte < 4; ++byte) bits |= uint32_t(cursor[i * 4 + byte]) << (byte * 8);
          std::memcpy(result.data() + i, &bits, 4);
        } else {
          result[i] = cv::float16_t::fromBits(uint16_t(cursor[i * 2]) | (uint16_t(cursor[i * 2 + 1]) << 8));
        }
      }
    cursor += bytes;
    remaining -= bytes;
    return result;
  }
};
}  // namespace sapphire::backend::feature
