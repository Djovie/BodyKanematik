cmake_minimum_required(VERSION 3.0.2)
project(sahdad)

add_compile_options(-std=c++14)

include_directories(
  ../src
)

add_library(dynamixel_workbench
    ../src/dynamixel_workbench_toolbox/dynamixel_item.cpp
    ../src/dynamixel_workbench_toolbox/dynamixel_driver.cpp
    ../src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp
    ../src/dynamixel_workbench_toolbox/dynamixel_tool.cpp
    ../src/dynamixel_workbench_toolbox/body.cpp
    ../src/dynamixel_workbench_toolbox/inverseKin.cpp
)

target_link_libraries(dynamixel_workbench LINK_PUBLIC "/usr/local/lib/libdxl_x64_cpp.so")

add_executable(gerak main.cpp)
target_link_libraries(gerak LINK_PUBLIC dynamixel_workbench)