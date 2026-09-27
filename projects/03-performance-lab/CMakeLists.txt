cmake_minimum_required(VERSION 3.20)

project(performance-lab LANGUAGES CXX)

add_library(performance_lab STATIC
  src/statistics.cpp
  src/bench_runner.cpp
)

target_include_directories(performance_lab PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_set_lab_warnings(performance_lab)

add_executable(bench_harness benchmarks/bench_harness.cpp)
target_link_libraries(bench_harness PRIVATE performance_lab)
target_set_lab_warnings(bench_harness)

add_executable(bench_cache_locality benchmarks/bench_cache_locality.cpp)
target_link_libraries(bench_cache_locality PRIVATE performance_lab)
target_set_lab_warnings(bench_cache_locality)

add_executable(bench_simd_vectorization benchmarks/bench_simd_vectorization.cpp)
target_link_libraries(bench_simd_vectorization PRIVATE performance_lab)
target_set_lab_warnings(bench_simd_vectorization)

add_executable(bench_lockfree_queues benchmarks/bench_lockfree_queues.cpp)
target_link_libraries(bench_lockfree_queues PRIVATE performance_lab)
target_set_lab_warnings(bench_lockfree_queues)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
