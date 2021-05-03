cmake_minimum_required(VERSION 3.19)
project(lab_03_md_kalman)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 14)

add_executable(lab_03_md_kalman main.cpp)
target_link_libraries (lab_03_md_kalman Eigen3::Eigen)
