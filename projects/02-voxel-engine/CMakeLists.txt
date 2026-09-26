cmake_minimum_required(VERSION 3.20)

project(voxel-engine LANGUAGES CXX)

add_library(voxel_engine STATIC
  src/chunk.cpp
  src/world_grid.cpp
  src/test_worlds.cpp
  src/naive_mesher.cpp
)

target_include_directories(voxel_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_set_lab_warnings(voxel_engine)

add_executable(demo_voxel_editing src/demo_voxel_editing.cpp)
target_link_libraries(demo_voxel_editing PRIVATE voxel_engine)
target_set_lab_warnings(demo_voxel_editing)

add_executable(bench_naive_mesher benchmarks/bench_naive_mesher.cpp)
target_link_libraries(bench_naive_mesher PRIVATE voxel_engine)
target_set_lab_warnings(bench_naive_mesher)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
