include(FetchContent)

if (APPLE)
  set(_ob_version "v2.4.3")
  set(_ob_commit "045a0e76")
  set(_ob_timestamp "202505192200")
  set(_ob_suffix "macOS_beta")
  set(_ob_sha256 "4f0ea5ae808321fc9703c20a47e10fc79197949552ca13bef809b462bd0bde70")
elseif (LINUX)
    # v2.7.2 is the first release that supports the Gemini 305.
    set(_ob_version "v2.10.6")
    set(_ob_commit "575347c")
    set(_ob_timestamp "202609291004")
    set(_ob_ext "tar.gz")

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
      set(_ob_suffix "linux_x86_64")
      set(_ob_sha256 "23ec9f7023881cc9061bf0b4ba65735fb16c10e168fd5d62b1548c847e7ddb46")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
      set(_ob_suffix "linux_arm64")
      set(_ob_sha256 "00fafc2b566ca98feb3ef0242143d830209b09bac5d41c557cf6ec40452c8df9")
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

if(NOT DEFINED _ob_ext)
  set(_ob_ext "zip")
endif()

set(_ob_sdk_dir "OrbbecSDK_${_ob_version}_${_ob_timestamp}_${_ob_commit}_${_ob_suffix}")

FetchContent_Declare(
    orbbec-sdk-release
    URL "https://github.com/orbbec/OrbbecSDK_v2/releases/download/${_ob_version}/${_ob_sdk_dir}.${_ob_ext}"
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
elseif(WIN32)
  # Copy dlls to the orbbecsdk lib folder
  file(COPY
  ${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/bin/orbbecsdk.dll
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/lib
  )
  file(COPY
  ${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/bin/extensions
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/lib
  )
endif()

set(OrbbecSDK_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/lib" CACHE PATH "Path to the Orbbec SDK" FORCE)
