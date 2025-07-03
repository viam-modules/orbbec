cmake_minimum_required(VERSION 3.10)
project(orbbec)


set(CMAKE_PROJECT_VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)
project(viam-orbbec
    DESCRIPTION "Viam module for orbbec cameras"
    HOMEPAGE_URL https://github.com/viam-modules/orbbec
    LANGUAGES CXX
)


add_executable(orbbec-cli src/cli/main.cpp)
add_executable(orbbec-module src/module/main.cpp)

set(OrbbecSDK_DIR  "${ORBBEC_SDK_DIR}/lib")
set(CMAKE_OSX_DEPLOYMENT_TARGET "14.5")

include_directories("${ORBBEC_SDK_DIR}/include")


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
