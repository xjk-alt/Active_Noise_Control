cmake_minimum_required(VERSION 3.14.0)

project(ancTest)
set(CMAKE_BUILD_TYPE Debug)
set(PROJECT_INCLUDE_DIRS ../ ../Controller/ ../src ../AncModel)

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

add_executable(exosysTest OscillatorTest.cpp)
target_include_directories(exosysTest PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(exosysTest ct_optcon)

add_executable(mtTest MTControllerTest.cpp)
target_include_directories(mtTest PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(mtTest ct_optcon)
