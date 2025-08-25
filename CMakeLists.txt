cmake_minimum_required(VERSION 3.10)
project(orbbec)

set(CMAKE_PROJECT_VERSION 0.0.1)
project(viam-orbbec
    DESCRIPTION "Viam module for orbbec cameras"
    HOMEPAGE_URL https://github.com/viam-modules/orbbec
    LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake")

add_executable(orbbec-cli src/cli/main.cpp)
add_executable(orbbec-module src/module/main.cpp src/module/orbbec.cpp src/module/discovery.cpp)

target_compile_features(orbbec-cli PRIVATE cxx_std_17)
target_compile_features(orbbec-module PRIVATE cxx_std_17)

if (APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "14.5")
endif()

include(fetch_orbbec)

find_package(OrbbecSDK REQUIRED)
find_package(viam-cpp-sdk REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(orbbec-cli
    PRIVATE ob::OrbbecSDK
    PRIVATE Threads::Threads
)

target_link_libraries(orbbec-module
    PRIVATE ob::OrbbecSDK
    PRIVATE Threads::Threads
    PRIVATE viam-cpp-sdk::viamsdk
)
