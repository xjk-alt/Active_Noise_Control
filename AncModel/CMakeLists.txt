cmake_minimum_required(VERSION 3.14.0)

project(AncModelSim)

set(PROJECT_INCLUDE_DIRS ../ ../Controller/ ../logger/ ../src)

find_package(Eigen3 REQUIRED)
find_package(msgpack REQUIRED)

find_package(ct_core REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)

add_executable(AncModelStepResponse AncModelStepResponse.cpp)
target_include_directories(AncModelStepResponse PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(AncModelStepResponse ct_optcon)

add_executable(AncModelFrequencyResponse AncModelFrequencyResponse.cpp)
target_include_directories(AncModelFrequencyResponse PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(AncModelFrequencyResponse ct_optcon)

add_executable(AncTFStepResponse AncTFStepResponse.cpp)
target_include_directories(AncTFStepResponse PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(AncTFStepResponse ct_optcon)
