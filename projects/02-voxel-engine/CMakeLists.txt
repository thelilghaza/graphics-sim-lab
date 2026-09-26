cmake_minimum_required(VERSION 3.20)

project(voxel-engine LANGUAGES CXX)

add_library(voxel_engine STATIC
  src/chunk.cpp
)

target_include_directories(voxel_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_set_lab_warnings(voxel_engine)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
