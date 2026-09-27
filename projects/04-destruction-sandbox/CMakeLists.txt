# Project 04: Procedural Destruction Sandbox
# CMake Build Configuration

cmake_minimum_required(VERSION 3.20)

project(destruction-sandbox LANGUAGES CXX)

message(STATUS "Configuring Project 04 — Procedural Destruction Sandbox (Milestone 4 Complete)")

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

# Milestone 2 Targets
add_executable(test_fracture_geometry tests/test_fracture_geometry.cpp)
target_link_libraries(test_fracture_geometry PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(test_fracture_geometry)
add_test(NAME test_fracture_geometry COMMAND test_fracture_geometry)

add_executable(demo_fracture tests/demo_fracture.cpp)
target_link_libraries(demo_fracture PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(demo_fracture)
add_test(NAME demo_fracture COMMAND demo_fracture)

# Milestone 3 Targets
add_executable(test_collision tests/test_collision.cpp)
target_link_libraries(test_collision PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(test_collision)
add_test(NAME test_collision COMMAND test_collision)

add_executable(demo_collision tests/demo_collision.cpp)
target_link_libraries(demo_collision PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(demo_collision)
add_test(NAME demo_collision COMMAND demo_collision)

# Milestone 4 Targets
add_executable(test_solver_graph tests/test_solver_graph.cpp)
target_link_libraries(test_solver_graph PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(test_solver_graph)
add_test(NAME test_solver_graph COMMAND test_solver_graph)

add_executable(demo_physics tests/demo_physics.cpp)
target_link_libraries(demo_physics PRIVATE destruction_sandbox_headers)
target_set_lab_warnings(demo_physics)
add_test(NAME demo_physics COMMAND demo_physics)

# Planned future targets (Milestone 5):
# Milestone 5: sandbox_demo, bench_destruction_physics
