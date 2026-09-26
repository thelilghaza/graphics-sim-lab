cmake_minimum_required(VERSION 3.20)

function(add_voxel_test TEST_NAME SOURCES)
  add_executable(${TEST_NAME} ${SOURCES})
  target_include_directories(${TEST_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )
  target_set_lab_warnings(${TEST_NAME})
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

add_voxel_test(test_voxel test_voxel.cpp)
add_voxel_test(test_chunk "test_chunk.cpp;../src/chunk.cpp")
add_voxel_test(test_coordinates test_coordinates.cpp)
add_voxel_test(test_world_accessor "test_world_accessor.cpp;../src/chunk.cpp;../src/world_grid.cpp")
add_voxel_test(test_voxel_editing "test_voxel_editing.cpp;../src/chunk.cpp;../src/world_grid.cpp")
add_voxel_test(test_test_worlds "test_test_worlds.cpp;../src/chunk.cpp;../src/world_grid.cpp;../src/test_worlds.cpp")
add_voxel_test(test_naive_mesher "test_naive_mesher.cpp;../src/chunk.cpp;../src/world_grid.cpp;../src/test_worlds.cpp;../src/naive_mesher.cpp")
add_voxel_test(test_voxel_camera "test_voxel_camera.cpp;../src/camera.cpp")
