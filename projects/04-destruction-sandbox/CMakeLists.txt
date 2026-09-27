# Project 04: Procedural Destruction Sandbox
# CMake Build Configuration (Phase 0 Skeleton)

cmake_minimum_required(VERSION 3.20)

project(destruction-sandbox LANGUAGES CXX)

message(STATUS "Configuring Project 04 — Procedural Destruction Sandbox (Phase 0 Planning Complete)")

# Header-only include directory configuration for Phase 0 planning stage
add_library(destruction_sandbox_headers INTERFACE)
target_include_directories(destruction_sandbox_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Planned future targets (to be enabled during implementation milestones):
#
# Milestone 1:
# add_library(destruction_sandbox STATIC ...)
# add_executable(test_destruction_math ...)
# add_executable(test_destruction_dynamics ...)
#
# Milestone 2:
# add_executable(test_destruction_fracture ...)
# add_executable(bench_destruction_fracture ...)
#
# Milestone 3:
# add_executable(test_destruction_collision ...)
#
# Milestone 4:
# add_executable(test_destruction_solver ...)
# add_executable(test_destruction_graph ...)
#
# Milestone 5:
# add_executable(sandbox_demo ...)
# add_executable(bench_destruction_physics ...)
