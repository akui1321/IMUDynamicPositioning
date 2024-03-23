cmake_minimum_required(VERSION 3.0)

project(IMUDynamicPositioning)

add_subdirectory(src)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(PositioningExecutable main.cpp)

target_link_libraries(PositioningExecutable PRIVATE EulerAnglesCalculator IMUDataReader Plotter CrossProduct Updating Magnitude MatrixToEuler BLtoNE)
