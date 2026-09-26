cmake_minimum_required(VERSION 3.20)

project(graphics-sim-lab
  VERSION 0.1.0
  DESCRIPTION "Graphics & Simulation Lab - Personal Technical Portfolio"
  LANGUAGES CXX
)

# Enforce C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable testing framework globally
include(CTest)

# Compiler Warning Flags (Strict, but non-fatal during repository bootstrap)
function(target_set_lab_warnings TARGET_NAME)
  if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /W4)
  else()
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# Include project subdirectories conditionally if CMakeLists.txt is present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/projects/01-raytracer/CMakeLists.txt")
  add_subdirectory(projects/01-raytracer)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/projects/02-voxel-engine/CMakeLists.txt")
  add_subdirectory(projects/02-voxel-engine)
endif()

# Print foundation status summary
message(STATUS "--------------------------------------------------------")
message(STATUS "Graphics & Simulation Lab Foundation Configured")
message(STATUS "  C++ Standard : ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler ID  : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Testing      : Enabled")
message(STATUS "--------------------------------------------------------")
