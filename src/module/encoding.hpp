#pragma once
#include <png.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace orbbec {
namespace encoding {

std::vector<std::uint8_t> encode_to_depth_raw(std::uint8_t const* const data, std::uint32_t const width, std::uint32_t const height);
std::vector<std::uint8_t> encode_to_png(std::uint8_t const* const image_data, std::uint32_t const width, std::uint32_t const height);
std::vector<std::uint8_t> encode_grayscale_to_png(std::uint8_t const* const image_data,
                                                  std::uint32_t const width,
                                                  std::uint32_t const height);
std::vector<std::uint8_t> encode_depth_to_png(std::uint8_t const* const data,
                                              std::uint32_t const width,
                                              std::uint32_t const height,
                                              float value_scale,
                                              float depth_min_mm,
                                              float depth_max_mm);
}  // namespace encoding
}  // namespace orbbec
