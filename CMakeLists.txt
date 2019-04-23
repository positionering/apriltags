cmake_minimum_required(VERSION 3.1)
project(apriltag)

set (CMAKE_C_FLAGS "-std=gnu99 ${CMAKE_C_FLAGS}")

FIND_PACKAGE(PkgConfig)
pkg_check_modules(APRIL REQUIRED apriltag)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(apriltags apriltags.cc)

target_link_libraries(apriltags apriltag ${OpenCV_LIBRARIES})
