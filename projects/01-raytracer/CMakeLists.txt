cmake_minimum_required(VERSION 3.20)

project(raytracer LANGUAGES CXX)

add_executable(raytracer
  src/main.cpp
  src/camera.cpp
  src/sphere.cpp
  src/hittable_list.cpp
  src/bvh_node.cpp
  src/ppm_writer.cpp
)

target_include_directories(raytracer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_set_lab_warnings(raytracer)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
