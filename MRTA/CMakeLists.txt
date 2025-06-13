cmake_minimum_required(VERSION 3.0.0)
project(MRTA VERSION 0.1.0 LANGUAGES C CXX)
set (CMAKE_CXX_STANDARD 14)

# 소스 디렉토리 경로를 포함하도록 설정
include_directories(${CMAKE_SOURCE_DIR})

# 새로운 소스 파일 추가
file(WRITE ${CMAKE_BINARY_DIR}/robot_static_members.cpp
"#include \"simulator.h\"
// 정적 멤버 변수 정의
constexpr int ROBOT::view_range_list[];
constexpr ROBOT::VIEWTYPE ROBOT::view_type_list[];
constexpr int ROBOT::energy_per_tick_list[];
")

# robot_static_members.cpp 파일을 추가하여 정적 멤버 정의
add_executable(MRTA main.cpp simulator.h simulator.cpp schedular.h schedular.cpp ${CMAKE_BINARY_DIR}/robot_static_members.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})