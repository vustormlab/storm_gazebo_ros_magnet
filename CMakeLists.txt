cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(storm_gazebo_magnet)

  
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  gazebo_plugins
)  
  
find_package(gazebo REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_PLUGINS_INCLUDE_DIRS}
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
# set (CMAKE_CXX_FLAGS "-std=c++11")
add_definitions(-std=c++11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -O2")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs roscpp gazebo_plugins
  DEPENDS gazebo
)

add_library(storm_gazebo_dipole_magnet src/dipole_magnet.cc)
add_dependencies(storm_gazebo_dipole_magnet ${catkin_EXPORTED_TARGETS})
target_link_libraries(storm_gazebo_dipole_magnet ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  
