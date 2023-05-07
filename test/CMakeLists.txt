cmake_minimum_required(VERSION 3.14.0)

project(ancTest)

set(PROJECT_INCLUDE_DIRS ../ ../Controller/ ../src)

find_package(Eigen3 REQUIRED)
find_package(msgpack REQUIRED)

find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)

add_executable(tfTest tfTest.cpp)
target_include_directories(tfTest PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(tfTest ct_optcon)

