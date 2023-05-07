cmake_minimum_required(VERSION 3.14.0)

project(Experiments)

set(CMAKE_BUILD_TYPE Debug)

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

add_executable(AncModelMTController AncModelMTController.cpp)
target_include_directories(AncModelMTController PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(AncModelMTController ct_optcon)

