cmake_minimum_required(VERSION 3.11)
project(stereo-recorder VERSION 0.0.1 LANGUAGES CXX)

# 3rdparty Libraries

SET(CMAKE_COLOR_MAKEFILE ON)
SET(CMAKE_VERBOSE_MAKEFILE OFF)

add_subdirectory(3rd_party)
add_subdirectory(src)
