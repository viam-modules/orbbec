#include "encoding.hpp"
#include <cmath>
#include <viam/sdk/components/camera.hpp>
namespace orbbec {
namespace encoding {

std::vector<std::uint8_t> encode_to_depth_raw(std::uint8_t const* const data, std::uint32_t const width, std::uint32_t const height) {
    viam::sdk::Camera::depth_map m = xt::xarray<uint16_t>::from_shape({height, width});
    std::copy(reinterpret_cast<const uint16_t*>(data), reinterpret_cast<const uint16_t*>(data) + height * width, m.begin());

    return viam::sdk::Camera::encode_depth_map(m);
}

std::vector<std::uint8_t> encode_to_png(std::uint8_t const* const image_data, std::uint32_t const width, std::uint32_t const height) {
    std::vector<std::uint8_t> png_buffer;

    // PNG write callback function
    auto png_write_callback = [](png_structp png_ptr, png_bytep data, png_size_t length) {
        auto* buffer = static_cast<std::vector<std::uint8_t>*>(png_get_io_ptr(png_ptr));
        buffer->insert(buffer->end(), data, data + length);
    };

    // Create PNG write struct
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        throw std::runtime_error("Could not create PNG write struct");
    }

    // Create PNG info struct
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        throw std::runtime_error("Could not create PNG info struct");
    }

    // Set up error handling
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        throw std::runtime_error("Error during PNG creation");
    }

    // Set custom write function
    png_set_write_fn(png_ptr, &png_buffer, png_write_callback, nullptr);

    // For now we are only supporting RGB images, so we set color type to RGB. In the future we can add support for RGBA and GRAY.
    int color_type = PNG_COLOR_TYPE_RGB;

    // Write PNG header
    png_set_IHDR(
        png_ptr, info_ptr, width, height, 8, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, info_ptr);

    // Write image data row by row
    int const RGB_CHANNELS = 3;
    std::vector<png_bytep> row_pointers(height);
    for (int y = 0; y < height; y++) {
        row_pointers[y] = const_cast<png_bytep>(&image_data[y * width * RGB_CHANNELS]);
    }

    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, nullptr);

    // Cleanup
    png_destroy_write_struct(&png_ptr, &info_ptr);

    return png_buffer;
}

std::vector<std::uint8_t> encode_grayscale_to_png(std::uint8_t const* const image_data,
                                                  std::uint32_t const width,
                                                  std::uint32_t const height) {
    std::vector<std::uint8_t> png_buffer;

    auto png_write_callback = [](png_structp png_ptr, png_bytep data, png_size_t length) {
        auto* buffer = static_cast<std::vector<std::uint8_t>*>(png_get_io_ptr(png_ptr));
        buffer->insert(buffer->end(), data, data + length);
    };

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        throw std::runtime_error("Could not create PNG write struct");
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        throw std::runtime_error("Could not create PNG info struct");
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        throw std::runtime_error("Error during PNG creation");
    }

    png_set_write_fn(png_ptr, &png_buffer, png_write_callback, nullptr);

    png_set_IHDR(png_ptr,
                 info_ptr,
                 width,
                 height,
                 8,
                 PNG_COLOR_TYPE_GRAY,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, info_ptr);

    std::vector<png_bytep> row_pointers(height);
    for (std::uint32_t y = 0; y < height; y++) {
        row_pointers[y] = const_cast<png_bytep>(&image_data[y * width]);
    }

    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, nullptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);

    return png_buffer;
}

// Apply JET colormap to an 8-bit value (0-255), writing R, G, B into out[0..2].
// Matches the OpenCV COLORMAP_JET piecewise linear approximation.
static void jet_colormap(uint8_t v, uint8_t* out) {
    float t = v / 255.0f;
    float r = std::clamp(1.5f - std::abs(4.0f * t - 3.0f), 0.0f, 1.0f);
    float g = std::clamp(1.5f - std::abs(4.0f * t - 2.0f), 0.0f, 1.0f);
    float b = std::clamp(1.5f - std::abs(4.0f * t - 1.0f), 0.0f, 1.0f);
    out[0] = static_cast<uint8_t>(r * 255.0f);
    out[1] = static_cast<uint8_t>(g * 255.0f);
    out[2] = static_cast<uint8_t>(b * 255.0f);
}

std::vector<std::uint8_t> encode_depth_to_png(std::uint8_t const* const data,
                                              std::uint32_t const width,
                                              std::uint32_t const height,
                                              float value_scale,
                                              float depth_min_mm,
                                              float depth_max_mm) {
    const uint16_t* src = reinterpret_cast<const uint16_t*>(data);
    // Normalize Y16 depth to 8-bit: map depth_min_mm-depth_max_mm to 0-255,
    // apply gamma 0.6 for near-object contrast, then apply JET colormap.
    float const range = depth_max_mm - depth_min_mm;
    std::vector<uint8_t> rgb(width * height * 3);
    for (uint32_t i = 0; i < width * height; i++) {
        float v = (src[i] * value_scale - depth_min_mm) / range * 256.0f;
        v = std::pow(v, 0.6f);
        v = std::clamp(v * 10.0f, 0.0f, 255.0f);
        jet_colormap(static_cast<uint8_t>(v), &rgb[i * 3]);
    }
    return encode_to_png(rgb.data(), width, height);
}

}  // namespace encoding
}  // namespace orbbec
