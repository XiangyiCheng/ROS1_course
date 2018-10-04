cmake_minimum_required(VERSION 2.8.3)
project(new_ps2)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
#cs_add_executable(vel_cmd_service src/vel_cmd_service.cpp)
cs_add_executable(vel_cmd_client src/vel_cmd_client.cpp)
#cs_add_executable(my_minimal_controller src/my_minimal_controller.cpp)
#cs_add_executable(my_minimal_simulator src/my_minimal_simulator.cpp)
cs_add_executable(vel_cmd_service src/vel_cmd_service.cpp)
#cs_add_executable(vel_cmd_publisher src/vel_cmd_publisher.cpp)
cs_install()
cs_export()
    
