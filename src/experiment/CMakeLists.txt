set(flock3d_experiment_sources
    MetricsExport.cpp
    ExperimentRunner.cpp
)

set(flock3d_experiment_headers
    ${PROJECT_SOURCE_DIR}/include/flock3d/experiment/MetricsExport.hpp
    ${PROJECT_SOURCE_DIR}/include/flock3d/experiment/ExperimentRunner.hpp
)

add_library(flock3d_experiment
    ${flock3d_experiment_sources}
    ${flock3d_experiment_headers}
)
add_library(flock3d::experiment ALIAS flock3d_experiment)

target_include_directories(flock3d_experiment
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(flock3d_experiment PUBLIC flock3d::core)
flock3d_enable_warnings(flock3d_experiment)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "Source" FILES ${flock3d_experiment_sources})
source_group(TREE ${PROJECT_SOURCE_DIR}/include PREFIX "Public Headers" FILES ${flock3d_experiment_headers})

add_executable(flock3d_experiment_runner main.cpp)
target_link_libraries(flock3d_experiment_runner PRIVATE flock3d::experiment)
flock3d_enable_warnings(flock3d_experiment_runner)
