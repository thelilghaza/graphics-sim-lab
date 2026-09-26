cmake_minimum_required(VERSION 3.20)

project(voxel-engine LANGUAGES CXX)

if(BUILD_TESTING)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
  endif()
endif()
