cmake_minimum_required(VERSION 3.14.0)

project(AncModelSim)

set(PROJECT_INCLUDE_DIRS ../ ../Controller/ ../logger/)

find_package(Eigen3 REQUIRED)
find_package(msgpack REQUIRED)

find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)

add_executable(AncModelSim AncModelSim.cpp)
target_include_directories(AncModelSim PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(AncModelSim ct_optcon)

