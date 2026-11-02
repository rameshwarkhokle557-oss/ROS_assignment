cmake_minimum_required(VERSION 3.8)
project(rb_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/cmd_vel_relay.py DESTINATION lib/${PROJECT_NAME})

ament_package()
