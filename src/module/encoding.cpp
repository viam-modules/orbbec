#include "encoding.hpp"
#include <viam/sdk/components/camera.hpp>
namespace orbbec {
namespace encoding {

std::vector<std::uint8_t> encode_to_depth_raw(std::uint8_t const* const data,
                                              std::uint32_t const width,
                                              std::uint32_t const height,
                                              float valueScale) {
    // The Orbbec SDK returns raw uint16_t depth pixels whose unit depends on the sensor.
    // getValueScale() returns the multiplier needed to convert a raw pixel to millimeters:
    //   actual_distance_mm = raw_pixel * valueScale
    // The Viam depth map format stores uint16_t values in millimeters, so we apply the
    // scale and round to the nearest integer mm before encoding.
    viam::sdk::Camera::depth_map m = xt::xarray<uint16_t>::from_shape({height, width});
    const uint16_t* rawData = reinterpret_cast<const uint16_t*>(data);
    for (size_t i = 0; i < height * width; i++) {
        long mm = std::lround(rawData[i] * static_cast<double>(valueScale));
        m[i] = static_cast<uint16_t>(std::clamp(mm, 0L, static_cast<long>(std::numeric_limits<uint16_t>::max())));
    }
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

}  // namespace encoding
}  // namespace orbbec
