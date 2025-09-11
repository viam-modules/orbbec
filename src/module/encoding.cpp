#include "encoding.hpp"
#include <viam/sdk/components/camera.hpp>
namespace orbbec {
namespace encoding {

std::vector<std::uint8_t> encode_to_depth_raw(std::uint8_t const* const data, uint64_t const width, uint64_t const height) {
    viam::sdk::Camera::depth_map m = xt::xarray<uint16_t>::from_shape({height, width});
    std::copy(reinterpret_cast<const uint16_t*>(data), reinterpret_cast<const uint16_t*>(data) + height * width, m.begin());

    double const mmToMeterMultiple = 0.001;

    for (size_t i = 0; i < m.size(); i++) {
        m[i] = m[i] * mmToMeterMultiple;
    }

    return viam::sdk::Camera::encode_depth_map(m);
}

std::vector<std::uint8_t> encode_to_png(std::uint8_t const* const image_data, int const width, int const height, ImageFormat const format) {
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

    // Determine PNG color type based on channels
    int color_type;
    switch (format) {
        case ImageFormat::GRAY:
            color_type = PNG_COLOR_TYPE_GRAY;
            break;
        case ImageFormat::RGB:
            color_type = PNG_COLOR_TYPE_RGB;
            break;
        case ImageFormat::RGBA:
            color_type = PNG_COLOR_TYPE_RGBA;
            break;
        default:
            png_destroy_write_struct(&png_ptr, &info_ptr);
            throw std::runtime_error("Unsupported number of channels");
    }

    // Write PNG header
    png_set_IHDR(
        png_ptr, info_ptr, width, height, 8, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, info_ptr);

    // Write image data row by row
    std::vector<png_bytep> row_pointers(height);
    for (int y = 0; y < height; y++) {
        row_pointers[y] = const_cast<png_bytep>(&image_data[y * width * format_to_channels.at(format)]);
    }

    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, nullptr);

    // Cleanup
    png_destroy_write_struct(&png_ptr, &info_ptr);

    return png_buffer;
}

}  // namespace encoding
}  // namespace orbbec
