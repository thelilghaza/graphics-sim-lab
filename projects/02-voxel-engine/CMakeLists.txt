cmake_minimum_required(VERSION 3.20)

project(voxel-engine LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

add_library(voxel_engine STATIC
  src/chunk.cpp
  src/world_grid.cpp
  src/test_worlds.cpp
  src/naive_mesher.cpp
  src/greedy_mesher.cpp
  src/lod.cpp
  src/thread_pool.cpp
  src/chunk_snapshot.cpp
  src/chunk_manager.cpp
  src/camera.cpp
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

add_executable(bench_greedy_mesher benchmarks/bench_greedy_mesher.cpp)
target_link_libraries(bench_greedy_mesher PRIVATE voxel_engine)
target_set_lab_warnings(bench_greedy_mesher)

add_executable(bench_chunk_manager benchmarks/bench_chunk_manager.cpp)
target_link_libraries(bench_chunk_manager PRIVATE voxel_engine)
target_set_lab_warnings(bench_chunk_manager)

add_executable(bench_multithreading benchmarks/bench_multithreading.cpp)
target_link_libraries(bench_multithreading PRIVATE voxel_engine)
target_set_lab_warnings(bench_multithreading)

add_executable(bench_memory benchmarks/bench_memory.cpp)
target_link_libraries(bench_memory PRIVATE voxel_engine)
if(WIN32)
  target_link_libraries(bench_memory PRIVATE psapi)
endif()
target_set_lab_warnings(bench_memory)

add_executable(bench_lod benchmarks/bench_lod.cpp)
target_link_libraries(bench_lod PRIVATE voxel_engine)
if(WIN32)
  target_link_libraries(bench_lod PRIVATE psapi)
endif()
target_set_lab_warnings(bench_lod)

find_package(OpenGL REQUIRED)

add_executable(voxel_viewer
  src/gl_loader.cpp
  src/gl_mesh.cpp
  src/gl_shader.cpp
  src/scene_manager.cpp
  src/voxel_viewer.cpp
)
target_link_libraries(voxel_viewer PRIVATE
  voxel_engine
  glfw
  OpenGL::GL
)
target_set_lab_warnings(voxel_viewer)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
