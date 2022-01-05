cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

find_package(Eigen3 REQUIRED)

add_executable (Transformation main.cpp)
target_link_libraries(Transformation Eigen3::Eigen)