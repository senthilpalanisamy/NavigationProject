## Package Description ##
This package contains code for making the turtlebot follow waypoint

##  roslaunch command ##
1. `roslaunch nuturtle_robot test_movement.launch`
2. `roslaunch nuturtle_robot test_linear_movement.launch` 
3. `roslaunch nuturtle_robot follow_waypoints.launch`

## Files ##
1. rotation.cpp - A ros node for making the robot rotate in place
2. translation.cpp - A ros node for making the robot translate along its body x direction
3. turtle_interface.cpp - A c++ node that bridges internal logic with turtle bot topics and commands
4. turtle_waypoint.cpp - A c++ node for making the turtle follow waypoints
5. test_movement.launch - launch file for making the turtlebot rotate in place
6. test_linear_movement.launch - launch file for making the turtlebot translate along a linear direction
7. follow_waypoints.launch - launch file for making the turtlebot follow waypoints.
8. basic_remote.launch - A launch file for launching nodes in turtlebot for basic operation
