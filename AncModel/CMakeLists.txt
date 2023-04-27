cmake_minimum_required(VERSION 3.14.0)

project(AncModelSim)

set(PROJECT_INCLUDE_DIRS "")

find_package(Eigen3 REQUIRED)
list(APPEND PROJECT_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
list(APPEND PROJECT_INCLUDE_DIRS ../BlankController/)

find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)

add_executable(AncModelSim AncModelSim.cpp)
target_include_directories(AncModelSim PUBLIC ${PROJECT_INCLUDE_DIRS})
