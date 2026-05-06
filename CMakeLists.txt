cmake_minimum_required(VERSION 3.24)

project(flock3d
    VERSION 0.1.0
    DESCRIPTION "Real-time deterministic 3D boids simulation"
    LANGUAGES CXX
)

include(FetchContent)
include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

function(flock3d_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.0
    GIT_SHALLOW TRUE
)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

add_library(flock3d_core
    src/sim/BoidSimulation.cpp
    src/sim/BoidSimulation.hpp
    src/sim/SimulationParameters.hpp
    src/sim/SpatialHash3D.cpp
    src/sim/SpatialHash3D.hpp
    src/math/Vec3.hpp
)
add_library(flock3d::core ALIAS flock3d_core)
target_include_directories(flock3d_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(flock3d_core PUBLIC raylib)
flock3d_enable_warnings(flock3d_core)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX "Source" FILES
    src/sim/BoidSimulation.cpp
    src/sim/BoidSimulation.hpp
    src/sim/SimulationParameters.hpp
    src/sim/SpatialHash3D.cpp
    src/sim/SpatialHash3D.hpp
    src/math/Vec3.hpp
)

add_executable(flock3d
    src/app/main.cpp
    src/app/Application.cpp
    src/app/Application.hpp
    src/render/BoidRenderer.cpp
    src/render/BoidRenderer.hpp
)
target_link_libraries(flock3d PRIVATE flock3d::core raylib)
flock3d_enable_warnings(flock3d)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX "Source" FILES
    src/app/main.cpp
    src/app/Application.cpp
    src/app/Application.hpp
    src/render/BoidRenderer.cpp
    src/render/BoidRenderer.hpp
)

if(APPLE)
    target_link_libraries(flock3d PRIVATE "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
endif()

if(BUILD_TESTING)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(flock3d_tests
        tests/test_spatial_hash.cpp
        tests/test_simulation.cpp
    )
    target_link_libraries(flock3d_tests PRIVATE flock3d::core Catch2::Catch2WithMain)
    flock3d_enable_warnings(flock3d_tests)
    source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/tests PREFIX "Tests" FILES
        tests/test_spatial_hash.cpp
        tests/test_simulation.cpp
    )

    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
    include(Catch)
    catch_discover_tests(flock3d_tests)
endif()
