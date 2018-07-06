cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(BleSensorScanner)
find_package(gflags REQUIRED)
## TODO(tancred): Figure out the right way to require the bluetooth library
add_executable(scan src/scan.cc)
target_link_libraries(scan bluetooth)
target_link_libraries(scan gflags)
