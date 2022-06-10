cmake_minimum_required(VERSION 3.10)


set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set the project name
project(Pinocchio_Tutorial)

# add the executable
add_executable(load_model src/load_model.cpp)
add_executable(IK src/IK.cpp)


find_package(pinocchio REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIR})
target_link_libraries(load_model pinocchio::pinocchio)
target_link_libraries(IK pinocchio::pinocchio)