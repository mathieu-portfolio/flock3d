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

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Flock3DWarnings)
include(Flock3DDependencies)

flock3d_fetch_dependencies()

add_subdirectory(src)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
