# Project 04: Procedural Destruction Sandbox
# CMake Build Configuration

cmake_minimum_required(VERSION 3.20)

project(destruction-sandbox LANGUAGES CXX)

message(STATUS "Configuring Project 04 — Procedural Destruction Sandbox (Milestone 1 Complete)")

# Header-only include directory configuration
add_library(destruction_sandbox_headers INTERFACE)
target_include_directories(destruction_sandbox_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Milestone 1 Targets
add_executable(test_dynamics_math tests/test_dynamics_math.cpp)
target_link_libraries(test_dynamics_math PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(test_dynamics_math)
add_test(NAME test_dynamics_math COMMAND test_dynamics_math)

add_executable(val_dynamics_headless tests/val_dynamics_headless.cpp)
target_link_libraries(val_dynamics_headless PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(val_dynamics_headless)
add_test(NAME val_dynamics_headless COMMAND val_dynamics_headless)

# Planned future targets (Milestone 2+):
# Milestone 2: test_destruction_fracture, bench_destruction_fracture
# Milestone 3: test_destruction_collision
# Milestone 4: test_destruction_solver, test_destruction_graph
# Milestone 5: sandbox_demo, bench_destruction_physics
