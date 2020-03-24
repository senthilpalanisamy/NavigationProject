cmake_minimum_required(VERSION 3.5.4 FATAL_ERROR)
project(nuturtle_gazebo)

add_compile_options(-Wall -Wextra -Wno-psabi)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



find_package(catkin REQUIRED COMPONENTS rigid2d roscpp gazebo_ros nuturtlebot std_msgs)
find_package(gazebo REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nuturtle_robot
CATKIN_DEPENDS rigid2d roscpp gazebo_ros nuturtlebot std_msgs 
#  DEPENDS system_lib
)

# include_directories(${GAZEBO_INCLUDE_DIRS})
add_library(turtle_drive_plugin SHARED src/turtle_drive_plugin.cpp)

#add_executable(${PROJECT_NAME}_landmarks src/landmarks.cpp)
#add_library(circle_detection src/circle_detection.cpp)
#link_directories(${GAZEBO_LIBRARY_DIRS})

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


target_link_libraries(turtle_drive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_include_directories(turtle_drive_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
# target_link_libraries(circle_detection
#   ${catkin_LIBRARIES}
#   Eigen3::Eigen
# )
# include_directories(${catkin_INCLUDE_DIRS})


add_dependencies(turtle_drive_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# if (CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(circle_detection_unittest tests/circle_detection_test.cpp)
#   target_link_libraries(circle_detection_unittest  ${catkin_Libraries} gtest_main
#                         circle_detection)
# endif()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
