cmake_minimum_required(VERSION 3.8)
project(ras_logging)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(ras_logging)

ament_package()
