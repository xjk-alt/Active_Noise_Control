cmake_minimum_required(VERSION 3.3)

set(PROJECT_ROOT_DIR ../)

project(ancTest)
set(CMAKE_BUILD_TYPE Debug)
set(PROJECT_INCLUDE_DIRS ${PROJECT_ROOT_DIR} ${PROJECT_ROOT_DIR}/Controller/ ${PROJECT_ROOT_DIR}/src ${PROJECT_ROOT_DIR}/AncModel ${PROJECT_ROOT_DIR}/libs ${PROJECT_ROOT_DIR}/libs/Eigen ${PROJECT_ROOT_DIR}/libs/control-toolbox/ct_core/include)
set(LIBS_SRC ${PROJECT_ROOT_DIR}/libs/control-toolbox/ct_core/src/core/plot/plot.cpp)

# find_package(Eigen3 REQUIRED)
# find_package(msgpack REQUIRED)

add_executable(tfTest tfTest.cpp ${LIBS_SRC})
target_include_directories(tfTest PUBLIC ${PROJECT_INCLUDE_DIRS})

add_executable(exosysTest OscillatorTest.cpp ${LIBS_SRC})
target_include_directories(exosysTest PUBLIC ${PROJECT_INCLUDE_DIRS})

add_executable(mtTest MTControllerTest.cpp ${LIBS_SRC})
target_include_directories(mtTest PUBLIC ${PROJECT_INCLUDE_DIRS})
