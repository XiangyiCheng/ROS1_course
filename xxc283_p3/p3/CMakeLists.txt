cmake_minimum_required(VERSION 2.8.3)
project(p3)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(vel_cmd_actionserver src/vel_cmd_actionserver.cpp)
cs_add_executable(vel_cmd_actionclient src/vel_cmd_actionclient.cpp)

cs_install()
cs_export()
    
