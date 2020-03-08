cmake_minimum_required(VERSION 3.5.4 FATAL_ERROR)
project(nuturtle_gazebo)


find_package(catkin REQUIRED COMPONENTS rigid2d roscpp gazebo_ros nuturtlebot std_msgs)
find_package(gazebo REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nuturtle_robot
  CATKIN_DEPENDS rigid2d roscpp gazebo_ros nuturtlebot std_msgs
#  DEPENDS system_lib
)

# include_directories(${GAZEBO_INCLUDE_DIRS})
add_library(turtle_drive_plugin SHARED src/turtle_drive_plugin.cpp)
#link_directories(${GAZEBO_LIBRARY_DIRS})

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


target_link_libraries(turtle_drive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_include_directories(turtle_drive_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_dependencies(turtle_drive_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

