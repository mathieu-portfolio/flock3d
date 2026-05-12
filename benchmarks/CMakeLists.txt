add_executable(flock3d_neighbor_benchmark
    neighbor_count_benchmark.cpp
)
target_link_libraries(flock3d_neighbor_benchmark PRIVATE flock3d::core)
flock3d_enable_warnings(flock3d_neighbor_benchmark)

set(flock3d_benchmark_targets
    flock3d_spatial_hash_benchmark
    flock3d_simulation_update_benchmark
    flock3d_metrics_benchmark
    flock3d_noise_benchmark
    flock3d_aggregate_social_benchmark
    flock3d_simulation_ticks_benchmark
    flock3d_render_benchmark
)

add_executable(flock3d_spatial_hash_benchmark
    spatial_hash_benchmark.cpp
    benchmark_common.hpp
)
target_link_libraries(flock3d_spatial_hash_benchmark PRIVATE flock3d::core)

add_executable(flock3d_simulation_update_benchmark
    simulation_update_benchmark.cpp
    benchmark_common.hpp
)
target_link_libraries(flock3d_simulation_update_benchmark PRIVATE flock3d::core)

add_executable(flock3d_metrics_benchmark
    metrics_benchmark.cpp
    benchmark_common.hpp
)
target_link_libraries(flock3d_metrics_benchmark PRIVATE flock3d::core)

add_executable(flock3d_noise_benchmark
    noise_benchmark.cpp
    benchmark_common.hpp
)
target_link_libraries(flock3d_noise_benchmark PRIVATE flock3d::core)

add_executable(flock3d_aggregate_social_benchmark
    aggregate_social_benchmark.cpp
    benchmark_common.hpp
)
target_link_libraries(flock3d_aggregate_social_benchmark PRIVATE flock3d::core)

add_executable(flock3d_simulation_ticks_benchmark
    simulation_ticks_benchmark.cpp
    simulation_ticks_benchmark.hpp
)
target_link_libraries(flock3d_simulation_ticks_benchmark PRIVATE flock3d::core)

add_executable(flock3d_render_benchmark
    render_benchmark.cpp
    benchmark_common.hpp
)
target_link_libraries(flock3d_render_benchmark PRIVATE flock3d::render raylib)

foreach(target IN LISTS flock3d_benchmark_targets)
    flock3d_enable_warnings(${target})
endforeach()
