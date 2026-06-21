cmake_minimum_required(VERSION 3.5)
project(vel_publisher)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
        scripts/vel_publisher.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
