cmake_minimum_required(VERSION 3.10)
project(rover_vda5050_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fake_master.py scripts/fake_aux_io.py DESTINATION lib/${PROJECT_NAME})

ament_package()
