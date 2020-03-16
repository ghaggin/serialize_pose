cmake_minimum_required(VERSION 3.10)
project(serialize_pose)

add_library(serialize_pose src/serialize_pose.cpp)
target_include_directories(serialize_pose PUBLIC include/)

add_executable(run_test src/main.cpp)
target_include_directories(run_test PUBLIC include/)
target_link_libraries(run_test serialize_pose)