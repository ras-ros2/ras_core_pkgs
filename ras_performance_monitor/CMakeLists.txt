cmake_minimum_required(VERSION 3.8)
project(ras_performance_monitor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(ras_performance_monitor)

ament_package()
