cmake_minimum_required(VERSION 2.8.3)
project(p4)
find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(controller src/controller.cpp)
cs_add_executable(vel_publisher src/vel_publisher.cpp)
cs_install()
cs_export()
