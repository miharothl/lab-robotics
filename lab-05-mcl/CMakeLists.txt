cmake_minimum_required(VERSION 3.19)
project(lab_05_mcl)

set(CMAKE_CXX_STANDARD 14)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_executable(lab_05_mcl main.cpp include/matplotlibcpp.h)

target_link_libraries(lab_05_mcl ${PYTHON_LIBRARIES})
