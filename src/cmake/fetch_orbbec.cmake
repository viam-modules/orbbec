include(FetchContent)

if (APPLE)
  set(_ob_version "v2.4.3")
  set(_ob_commit "045a0e76")
  set(_ob_timestamp "202505192200")
  set(_ob_suffix "macOS_beta")
  set(_ob_sha256 "4f0ea5ae808321fc9703c20a47e10fc79197949552ca13bef809b462bd0bde70")
elseif (LINUX)
    set(_ob_version "v2.4.8")
    set(_ob_commit "ec8e3469")
    set(_ob_suffix "linux_${CMAKE_SYSTEM_PROCESSOR}")

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
      set(_ob_timestamp "202507031325")
      set(_ob_sha256 "4a42a1bfd0032389f3be00b125166013abef5fbaa0a2297493d062caa9cf4d0f")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
      set(_ob_timestamp "202507031330")
      set(_ob_sha256 "a9abe3fb783e284eb8c43843fd0aab202246ffe40212b9f9b9fdc86566239e82")
    else()
      message(FATAL_ERROR "Unsupported arch")
    endif()
elseif (WIN32)
  set(_ob_version "v2.4.8")
  set(_ob_commit "ec8e346")
  set(_ob_timestamp "202507032159")
  set(_ob_suffix "win_x64")
  set(_ob_sha256 "427f0f0596b5dcc229f10a78ad728da51e3065b599e2cc8737cbdedcc31f0aac")
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

set(_ob_sdk_dir "OrbbecSDK_${_ob_version}_${_ob_timestamp}_${_ob_commit}_${_ob_suffix}")

FetchContent_Declare(
    orbbec-sdk-release
    URL "https://github.com/orbbec/OrbbecSDK_v2/releases/download/${_ob_version}/${_ob_sdk_dir}.zip"
    URL_HASH "SHA256=${_ob_sha256}"
)

FetchContent_Makeavailable(orbbec-sdk-release)

if(LINUX)
  file(
    COPY_FILE
      ${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/shared/99-obsensor-libusb.rules
      ${CMAKE_CURRENT_SOURCE_DIR}/99-obsensor-libusb.rules
    ONLY_IF_DIFFERENT
  )
endif()

set(OrbbecSDK_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/lib" CACHE PATH "Path to the Orbbec SDK" FORCE)
