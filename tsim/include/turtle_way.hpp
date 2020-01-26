#ifndef TSIM_INCLUDE_GAURD_HPP 
#define TSIM_INCLUDE_GAURD_HPP

/// \file
///// \brief A RoS node for controlling a turtle in turtlesim package to
//           follow a rectangular path by using feed forward control
/////
///// PARAMETERS:
/////   x (float): x co-ordinate of the lower left corner of the rectangular
/////                 trajectory
/////      y (float): y co-ordinate of the lower left corner of the rectangular
/////                 trajectory
/////  width (float): Width of the rectangular trajectory
/////  height (float): Height of the rectangular trajectory
/////trans_vel(float): Linear velocity of the turtle 
/////  rot_vel(float): Angular velocity of the turtle 
/////frequency(float): Frequency of the node and the control loop 
///// PUBLISHES:
/////  /pose_error(tsim/ErrorPose): Publishes the error pose of the turtle
/////                               i.e., difference between the turtle's
/////                               belief about its pose and the actual
/////                               pose
///// SUBSCRIBES:
///// /turtle1/cmd_vel (turtlesim/Pose): The topic where the actual pose of the
/////                                     turtle is published by the turtlesim
/////                                     package.
///// SERVICES:
/////     /traj_reset (srd_srvs/Empty): A service for resetting the turtle to 
/////                                   its intial position i.e., to the lower
/////                                   left corner of the rectangular trajectory
/////  /turtle1/set_pen (turtlesim/SetPen): A service for adjusting the display
/////                                       properties of the turtle's pen and
/////                                       it can also be turned Off or ON
/////  /turtle1/teleport_absolute(turtlesim/TeleportAbsolute):
/////                                     A service from turtlesim which 
/////                                     allows to teleport the turtle to any
/////                                     location by specifying the absolute 
/////                                     value of the co-ordinates
/////

#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <std_srvs/Empty.h>
#include <turtlesim/Pose.h>
#include <vector>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <tsim/ErrorPose.h>

namespace TurtleWay
{

  /// \brief A enum to have descriptive form for all states used in the state
  ///         machine
  enum Turtle_States {INITIALISE, FOLLOW_WAYPOINT, CHANGE_WAY_POINT,
                      RESET};

  /// \brief A circular linked list implementation for switching between
  ///        subsequent waypoints
  struct Circular_Linked_List
  {
    Circular_Linked_List(Circular_Linked_List* previous_element, double x, double y);
    double x, y;
    Circular_Linked_List* next_element;
  };
  /// \brief A class holding all RoS communication setups and feedforward
  ///         control of the turtle.
  class Rect_Navigation
  {
    public:

      /// \brief This constructor initialises all RoS communications and 
      ///         parameters used throughout the package
      /// \param argc - Commandline argument from main. This is required for
      ///               initialising the node
      /// \param argv - Commandline argument from main. This is required for
      ///               intialising the node
      Rect_Navigation(int argc, char **argv);

      /// \brief The service call back function for restarting the turtle trajectory 
      void restart_trajectory();
      /// \brief The main logic resides in this function. It consists of logic
      ///        for switching between turtle states
      void execute_trajectory();

      /// \brief This function publishes velocities for making the turtle follow
      ///        a given waypoint
      void follow_way_point();

      /// \brief The service callback for resetting a turtle's trajectory. 
      /// \param request - An empty request
      /// \param response - An eimpty response
      bool reset_turtle_callback(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response);

      /// \brief A linked element that holds the current way point and the
      ///        first way point
      Circular_Linked_List *current_waypoint, *first_waypoint;

      /// \brief The state variable for the state machine
      Turtle_States state; 

      /// \brief Variables for holding the turtle's current state and goal state 
      double current_x, current_y, goal_angle, current_angle;

      /// \brief Frequency of the node and control loop execution 
      float frequency;
      /// \brief The previous velocity is stored for dead-reckoning / feedforward
      ///        calculation
      geometry_msgs::Twist previous_velocity;

      /// \brief The time stamp of the previous command is stored for deadreckoning
      ///        feedforward calculation
      ros::Time previous_time;

    private:

      /// \brief This function creates the circular linked list that contains
      ///        all waypoints for the turtle
      ///        an absolute comparison
      Circular_Linked_List* create_waypoints_list();
      /// \brief The callback function for subscribing to pose topic of turtle1
      ///        This is where the error pose if published.
      /// \param pose -The actual pose of the turtle published by the turtlesim
      ///              package. Its includes x,y position and orientation
      void pose_callback(const turtlesim::Pose pose);
      /// \brief Intialises the turtle in its initial position 
      void initialise_turtle();
      /// \brief  This function updates the turtle's belief about its pose
      ///         This is just a dead reckoning pose calculated without any
      ///         feedback
      void update_current_pose();

      /// \brief  Parameters defining the rectangular trajectory. These
      ///         are loaded from the ros server
      double rot_vel, trans_vel;
      std::vector<double> wayPointX, wayPointY;

      /// \brief  Some intialisation for establishing ros communications 
      ros::Publisher velocity_publisher, pose_error_publisher;
      ros::ServiceServer reset_turtle;
      ros::Subscriber pose_subscriber;
     
  };

}

#endif
