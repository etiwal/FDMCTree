cmake_minimum_required(VERSION 3.17)
project(MCSampling)

set(CMAKE_CXX_STANDARD 14)

add_executable(MCSampling main.cpp functions.cpp functions.h fdmcts.h fdmcts.cpp tree.h tree_util.h config.h config.cpp logger.h)

find_package(Python3 COMPONENTS Development)
target_include_directories(MCSampling PRIVATE ${Python3.8_INCLUDE_DIRS} ${Python3.8_NumPy_INCLUDE_DIRS})
target_link_libraries(MCSampling Python3::Python)# Python3.6::NumPy)

find_package(yaml-cpp REQUIRED)

