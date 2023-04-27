cmake_minimum_required(VERSION 3.14.0)

project(ct-test)

set(PROJECT_INCLUDE_DIRS "")

find_package(Eigen3 REQUIRED)
list(APPEND PROJECT_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)

add_executable(damped_oscillator oscillator/damped_oscillator.cpp)
target_include_directories(damped_oscillator PUBLIC ${PROJECT_INCLUDE_DIRS})

add_executable(mass_point mass_point/mass_point.cpp)
target_include_directories(mass_point PUBLIC ${PROJECT_INCLUDE_DIRS})

add_executable(custom_controller custom_controller/custom_controller.cpp)
target_include_directories(custom_controller PUBLIC ${PROJECT_INCLUDE_DIRS})