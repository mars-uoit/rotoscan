cmake_minimum_required(VERSION 2.8.3)
project(rotoscan_node)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs timeutils roslaunch)

find_package(Boost REQUIRED signals)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES rotoscan
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(rotoscan_node src/rotoscan_node.cc)
target_link_libraries(rotoscan_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Check the validity of the supplied launch files.
roslaunch_add_file_check(launch)
