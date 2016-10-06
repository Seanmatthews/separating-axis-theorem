cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_compile_options(-std=c++11)
project(sat)

find_package(PCL 1.8 REQUIRED COMPONENTS common surface io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sat main.cpp)
target_link_libraries(sat ${PCL_LIBRARIES})
