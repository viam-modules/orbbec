#pragma once
#include <png.h>

#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace orbbec {
namespace encoding {

// Both throw std::invalid_argument if the buffer is smaller than width and height require.
std::vector<std::uint8_t> encode_to_depth_raw(std::span<std::uint8_t const> depth_bytes,
                                              std::uint32_t const width,
                                              std::uint32_t const height,
                                              float valueScale);
std::vector<std::uint8_t> encode_to_png(std::span<std::uint8_t const> image_data, std::uint32_t const width, std::uint32_t const height);
}  // namespace encoding
}  // namespace orbbec
