cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(BleSensorScanner)

## TODO(tancred): Figure out the right way to require the libraries
add_executable(scan src/scan.c)
target_link_libraries(scan bluetooth)
