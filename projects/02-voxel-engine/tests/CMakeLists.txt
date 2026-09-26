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
