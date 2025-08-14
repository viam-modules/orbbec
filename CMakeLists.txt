cmake_minimum_required(VERSION 3.10)
project(orbbec)

set(CMAKE_PROJECT_VERSION 0.0.1)
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

target_compile_features(orbbec-cli PRIVATE cxx_std_17)
target_compile_features(orbbec-module PRIVATE cxx_std_17)

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

if (LINUX)
    set(_post_exclude_regex
	    ".*ld-linux.*\\.so.*"
	    ".*libc\\.so.*"
	    ".*libdl\\.so.*"
	    ".*libgcc.*\\.so.*"
	    ".*libm\\.so.*"
	    ".*libpthread\\.so.*"
    )
endif()

install(
    TARGETS orbbec-module
    RUNTIME_DEPENDENCIES
		POST_EXCLUDE_REGEXES ${_post_exclude_regex}
    RUNTIME
    LIBRARY
)

# The lib/extensions directory contains some shared libraries which are
# mandatory for our use case but not picked up by the runtime dependency set
# because they are loaded at runtime.
install(
    DIRECTORY ${OrbbecSDK_DIR}/extensions
    DESTINATION lib
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
