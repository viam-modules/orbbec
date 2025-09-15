#pragma once
#include <png.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace orbbec {
namespace encoding {

std::vector<std::uint8_t> encode_to_depth_raw(std::uint8_t const* const data, uint64_t const width, uint64_t const height);
std::vector<std::uint8_t> encode_to_png(std::uint8_t const* const image_data, int const width, int const height);
}  // namespace encoding
}  // namespace orbbec
