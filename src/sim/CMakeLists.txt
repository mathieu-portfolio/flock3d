set(flock3d_core_sources
    BoidSimulation.cpp
    NeighborSelection.cpp
    Scenario.cpp
    SpatialHash3D.cpp
)

set(flock3d_core_headers
    ${PROJECT_SOURCE_DIR}/include/flock3d/app/ControlCommands.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/app/ControlHelpers.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/app/FixedUpdateLoop.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/app/OverlayLayout.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/math/Vec3.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/sim/BoidSimulation.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/sim/NeighborSelection.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/sim/Scenario.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/sim/SimulationMetrics.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/sim/SimulationParameters.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/sim/SpatialHash3D.hpp
)

add_library(flock3d_core
    ${flock3d_core_sources}
    ${flock3d_core_headers}
)
add_library(flock3d::core ALIAS flock3d_core)

target_include_directories(flock3d_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(flock3d_core PUBLIC raylib)
flock3d_enable_warnings(flock3d_core)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "Source" FILES ${flock3d_core_sources})
source_group(TREE ${PROJECT_SOURCE_DIR}/include PREFIX "Public Headers" FILES ${flock3d_core_headers})
