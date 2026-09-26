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

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
