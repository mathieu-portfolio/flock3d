set(flock3d_app_sources
    main.cpp
    Application.cpp
    Application.hpp
)

add_executable(flock3d ${flock3d_app_sources})

target_include_directories(flock3d
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(flock3d
    PRIVATE
        flock3d::core
        flock3d::render
        raylib
)
flock3d_enable_warnings(flock3d)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "Source" FILES ${flock3d_app_sources})

if(APPLE)
    target_link_libraries(flock3d PRIVATE "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
endif()
