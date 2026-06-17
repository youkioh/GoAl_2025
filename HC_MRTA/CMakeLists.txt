cmake_minimum_required(VERSION 3.0.0)
project(MRTA VERSION 0.1.0 LANGUAGES C CXX)
set (CMAKE_CXX_STANDARD 14)

add_executable(MRTA main.cpp simulator.h simulator.cpp schedular.h schedular.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
