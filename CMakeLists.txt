cmake_minimum_required(VERSION 3.24)

project(flock3d
    VERSION 0.1.0
    DESCRIPTION "Real-time deterministic 3D boids simulation"
    LANGUAGES CXX
)

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Keep executable outputs deterministic across generators.
# This avoids stale binaries from older CMake target layouts.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

foreach(CONFIG Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${CONFIG}" CONFIG_UPPER)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_BINARY_DIR}/bin")
endforeach()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Flock3DWarnings)
include(Flock3DDependencies)

flock3d_fetch_dependencies()

add_subdirectory(src)
add_subdirectory(benchmarks)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()