set(flock3d_test_sources
    test_spatial_hash.cpp
    test_simulation.cpp
)

add_executable(flock3d_tests ${flock3d_test_sources})
target_link_libraries(flock3d_tests PRIVATE flock3d::core Catch2::Catch2WithMain)
flock3d_enable_warnings(flock3d_tests)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "Tests" FILES ${flock3d_test_sources})

include(Catch)
catch_discover_tests(flock3d_tests)
