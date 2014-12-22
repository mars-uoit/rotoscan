cmake_minimum_required(VERSION 2.8.3)
project(timeutils)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
