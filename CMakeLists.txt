cmake_minimum_required(VERSION 3.17)
project(MCSampling)

set(CMAKE_CXX_STANDARD 14)

add_executable(MCSampling src/main.cpp src/functions.cpp src/functions.h src/node.h src/node.cpp lib/tree.h lib/tree_util.h src/config.h src/config.cpp src/logger.h src/mulitvariate_normal_eigen.h src/gaussian_sampler.h src/sys_sim.h src/sys_sim.cpp src/trajects.h src/trajects.cpp src/occupancygrid.h src/occupancygrid.cpp src/expert.h src/expert.cpp src/expertbase.h src/normexp.h src/normexp.cpp src/impexp.h src/impexp.cpp)

find_package(Python3 COMPONENTS Development)
target_include_directories(MCSampling PRIVATE ${Python3.8_INCLUDE_DIRS} ${Python3.8_NumPy_INCLUDE_DIRS})
target_link_libraries(MCSampling Python3::Python)# Python3.6::NumPy)

find_package(yaml-cpp REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

