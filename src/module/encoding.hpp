#pragma once
#include <png.h>

#include <vector>
#include <unordered_map>
#include <cstdint>

namespace orbbec {
namespace encoding {

enum class ImageFormat {
    RGB,
    RGBA,
    GRAY
};

static const std::unordered_map<ImageFormat, int> format_to_channels = {
    {ImageFormat::RGB, 3},
    {ImageFormat::RGBA, 4},
    {ImageFormat::GRAY, 1}
};

std::vector<std::uint8_t> encode_to_depth_raw(std::uint8_t const * const  data, uint64_t const width, uint64_t const height);
std::vector<std::uint8_t> encode_to_png(std::uint8_t const * const image_data, int const width, int const height, ImageFormat const format);
} // namespace encoding
} // namespace orbbec
