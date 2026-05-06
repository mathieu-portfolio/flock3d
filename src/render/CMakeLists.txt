set(flock3d_render_sources
    BoidRenderer.cpp
    BoidRenderer.hpp
)

add_library(flock3d_render STATIC ${flock3d_render_sources})
add_library(flock3d::render ALIAS flock3d_render)

target_include_directories(flock3d_render
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(flock3d_render
    PUBLIC
        flock3d::core
    PRIVATE
        raylib
)
flock3d_enable_warnings(flock3d_render)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "Source" FILES ${flock3d_render_sources})
