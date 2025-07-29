include(FetchContent)

if (APPLE)
  set(_ob_version "v2.4.3")
  set(_ob_commit "045a0e76")
  set(_ob_timestamp "202505192200")
  set(_ob_suffix "macOS_beta")
  set(_ob_sha256 "4f0ea5ae808321fc9703c20a47e10fc79197949552ca13bef809b462bd0bde70")
elseif (UNIX)
   set(_ob_version "v2.4.8")
   set(_ob_commit "ec8e3469")

   # TODO use arch to form name
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

set(OrbbecSDK_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/lib" CACHE PATH "Path to the Orbbec SDK" FORCE)
