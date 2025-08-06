cmake_minimum_required(VERSION 3.10)
project(orbbec)

set(CMAKE_PROJECT_VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)
project(viam-orbbec
    DESCRIPTION "Viam module for orbbec cameras"
    HOMEPAGE_URL https://github.com/viam-modules/orbbec
    LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake")

# Here we set the rpath to accomodate one and only one supported deployment
# case, a directory containing
# ├── bin
# │   └── orbbec-module
# ├── lib
#     └── <runtime dependencies>
if (APPLE)
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()

add_executable(orbbec-cli src/cli/main.cpp)
add_executable(orbbec-module src/module/main.cpp src/module/orbbec.cpp src/module/discovery.cpp)

set(CMAKE_OSX_DEPLOYMENT_TARGET "14.5")

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


install(
    TARGETS orbbec-module
    DESTINATION bin
)

install(
    DIRECTORY
    ${OrbbecSDK_DIR}
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

install(
    FILES
        meta.json
        99-obsensor-libusb.rules
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

install(
    PROGRAMS
        first_run.sh
        install_udev_rules.sh
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)
