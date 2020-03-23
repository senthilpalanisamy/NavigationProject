/// \file
/// \brief This file implements a node that publishes cmd_vel for making the turtlebot
///        follow a sequence of way points.
///
/// PARAMETERS:
/// wayPointX (vector<double>) - x co-ordinates of all waypoints to be followed
/// wayPointY (vector<double>) - y co-ordinates of all waypoints to be followed
/// frequency (double)         - The frequency at which the node should run
/// fracVel (double)           - The fraction of the maximum velocity that could be used.
/// maxTransVelRobot (double)  - Maximum translation velocity of the robot
/// maxRotVelRobot (double)    - Maximum rotation velocity of the robot
/// ~px (double)               - proportional constant for linear velocity control
/// ~pw (double)               - proportional constant for angular velocity control
/// tolerance (double)         - The maximum distance between the current turtlebot location
///                              and the goal point so that the goal can be considered to 
///                              be reached.
/// PUBLISHES:
/// cmd_vel (geometry_msgs/Twist) - The body twist of the robot to be followed for following
///                                 the waypoint trajectory
/// visualization_marker (visualization_msgs/Marker) - Publishes a marker for each waypoint
///                                                    so that the waypoint being followed
///                                                    are clearly represented in rviz
/// SUBSCRIBES:
/// nav_msgs/Odometry (nav_msgs/Odometry) - Reads the current pose of the robot from the
///                                         odometry published by the odometry node
/// SERVICES:
/// start (std_srvs/Empty) - A service for beginning the whole process. This starts the
///                          cmd_vel publishing
/// stop (std_srvs/Empty)  - A service for stopping the whole process. This stops the 
///                          cmd vel publishing and finishes the execution of the node
/// set_pose (rigid2d/SetPose) - A service for setting the pose of the robot to the desired
///                              pose
/// /fake/set_pose (rigid2d/SetPose) - A service for setting the pose of the fake odometry
///                                   node to the desired position
#include <ros/ros.h>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Twist.h> 
#include <std_srvs/Empty.h>
#include "rigid2d/SetPose.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>


class WaypointTurtle
{

  private:
  std::vector<double> wayPointX, wayPointY;
  ros::Publisher velocity_publisher, marker_pub;
  ros::ServiceClient setPoseClient, setPoseFake;
  ros::Subscriber odometrySubscriber;
  double rotVel, transVel, maxTransVelRobot, maxRotVelRobot, fracVel, tolerance;

  enum Turtle_States {INITIALISE, WAIT, FOLLOW_WAYPOINT, CHANGE_WAY_POINT,
                      STOP};
  Turtle_States state;
  ros::ServiceServer startTrajectory, stopTrajectory;
  double current_x, current_y, goal_angle, current_angle;
  rigid2d::Vector2D current_waypoint;
  size_t wayPointIndex;
  std::vector<rigid2d::Vector2D> trajPoints;
  double linearp, angularp;
  ros::Time previous_time;
  bool isAllPointsVisited;
   visualization_msgs::Marker marker;

  public:
  double frequency;
  /// \brief A parameterized constructor for initializing the object.
  /// \param argc - Number of command line arguments
  /// \param argv - Command line arguments
  WaypointTurtle(int argc, char **argv)
  {

  ros::init(argc, argv, "waypointTurtle");
  ros::NodeHandle n;


   if (not ros::param::get("waypoint_x", wayPointX))
     {
      ROS_ERROR("Failed to get param waypoint_x");
     }

  if (not ros::param::get("waypoint_y", wayPointY))
    {
      ROS_ERROR("Failed to get param waypoint_y");
    }

  if (not ros::param::get("frequency", frequency))
    {
      ROS_ERROR("Failed to get param frequency");
    }

  if (not ros::param::get("~frac_vel", fracVel))
    {
      ROS_ERROR("Failed to get param fracVel");
    }

  if (not ros::param::get("max_trans_vel", maxTransVelRobot))
    {
      ROS_ERROR("Failed to get param maxTransVelRot");
    }

  if (not ros::param::get("max_rot_vel_robot", maxRotVelRobot))
    {
      ROS_ERROR("Failed to get param maxTransVelRot");
    }

  if (not ros::param::get("~px",  linearp))
   {
     ROS_ERROR("Failed to get param fracVel");
   }

  if (not ros::param::get("~pw",  angularp))
   {
     ROS_ERROR("Failed to get param fracVel");
   }

  if (not ros::param::get("tolerance",  tolerance))
   {
     ROS_ERROR("Failed to get param tolerance");
   }

  for(size_t i=0; i< wayPointX.size(); i++)
  {
    rigid2d::Vector2D point = {wayPointX[i], wayPointY[i]};
    trajPoints.push_back(point);
    ROS_INFO_STREAM("pointX"<<point.x<<"pointY"<<point.y);
  }
  ROS_INFO_STREAM("waypointsize"<<trajPoints.size());

  state = INITIALISE;
  velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  startTrajectory = n.advertiseService("/start", &WaypointTurtle::startWaypointFollowing,
                                     this);

  stopTrajectory = n.advertiseService("/stop", &WaypointTurtle::stopWaypointFollowing,
                                     this);

  setPoseClient = n.serviceClient<rigid2d::SetPose>("set_pose");
  //setPoseFake = n.serviceClient<rigid2d::SetPose>("/fake/set_pose");
  odometrySubscriber = n.subscribe("/nav_msgs/odometry", 1000, &WaypointTurtle::odometryCallback,
                                    this);
  rotVel = fracVel * maxRotVelRobot; 
  transVel = fracVel * maxTransVelRobot;
  wayPointIndex = 0;
  isAllPointsVisited = false;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100,true);
  }

  /// \brief A call back function for odometry message. This callback reads the odometry
  ///        and updates the pose of the robot within the object
  /// \param odometryMessage - Message about the odometry of the robot

  void odometryCallback(const nav_msgs::Odometry odometryMessage)
  {

    current_x = odometryMessage.pose.pose.position.x;
    current_y = odometryMessage.pose.pose.position.y;
    tf::Quaternion q(
        odometryMessage.pose.pose.orientation.x,
        odometryMessage.pose.pose.orientation.y,
        odometryMessage.pose.pose.orientation.z,
        odometryMessage.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw); 
    current_angle = yaw;

  }

  /// \brief A service for beginning the process. This service initiates the publishing
  ///        of cmd_vel velocities
  /// \param request - An empty request message
  /// \param response - An empty response
  bool startWaypointFollowing(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response)
  {

  state = FOLLOW_WAYPOINT;
  publish_marker();
  return true;
  }

  /// \brief This service stops the publishing of cmd_vel and changes the state of the node
  ///        to stop state, which will force the robot to come a halt immediately.
  /// \param request - An empty request
  /// \param response - An empty response
  bool stopWaypointFollowing(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response)
  {

  state = STOP;
  return true;
  }


  /// \brief This function publishes cmd_vel so that the robot could continue towards a 
  ///        waypoint and changes the state of the robot when the waypoint is reached
  void follow_way_point()
  {

  geometry_msgs::Twist velocity_message;

  double goal_angle = atan2(current_waypoint.y - current_y, current_waypoint.x - current_x);
  double angle_difference = current_angle - goal_angle;
  double angle_diff_wrapped = atan2(sin(angle_difference), cos(angle_difference));
  double distance_difference = pow(pow((current_waypoint.x - current_x), 2) +
                              pow((current_waypoint.y - current_y), 2), 0.5);


  if(abs(angle_diff_wrapped) >= tolerance)
  {
    ROS_INFO_STREAM("rotVel"<<rotVel<<"angularp"<<angularp);
     if (angle_diff_wrapped > 0)
       velocity_message.angular.z =  - angularp * rotVel;
     else
      velocity_message.angular.z =  angularp * rotVel;
    velocity_publisher.publish(velocity_message);
  } 
  else if (distance_difference > 0.1)
  {

    ROS_INFO_STREAM("transVel"<<transVel<<"linearp"<<linearp);
    ROS_INFO_STREAM("x"<<current_x<<"y"<<current_y);
    ROS_INFO_STREAM("waypoint_x"<<current_waypoint.x<<"waypoint_y"<<current_waypoint.y);

    velocity_message.linear.x = linearp * transVel;
    velocity_publisher.publish(velocity_message);
  } 
  else
  {
    state = CHANGE_WAY_POINT;
  }
  previous_time = ros::Time::now();


  }
 
  /// \brief This function publishes markers for waypoints
  void publish_marker()
  {
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = tolerance;
    marker.scale.y = tolerance;
    marker.scale.z = 0.2;
  
     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 0.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
     marker.lifetime = ros::Duration(0.0);
  
     while (marker_pub.getNumSubscribers() < 1)
     {
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }
  
    for(size_t i=0; i< trajPoints.size(); i++)
    {
  
     marker.ns = "basic_shapes";
     marker.id = i;
     marker.pose.position.x = trajPoints[i].x;
     marker.pose.position.y = trajPoints[i].y;
     marker.pose.position.z = 0;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     marker_pub.publish(marker);
    }
  
  
  
  }

  /// \brief This function initialises settings for the robot, publishes a few dummy
  /// messages in publisher topics, call services to initialise pose setup 

  void initialise_settings()
  {

  current_x = trajPoints[wayPointIndex].x;
  current_y = trajPoints[wayPointIndex].y;
  current_angle = 0.0;
  wayPointIndex += 1;
  current_waypoint = trajPoints[wayPointIndex];

  ROS_INFO_STREAM("Calling Service");
  ros::service::waitForService("/set_pose", -1);
  //ros::service::waitForService("fake/set_pose", -1);
  if(setPoseClient.exists())
  {

    ROS_INFO_STREAM("Entered service client");
    rigid2d::SetPose initialPose;
    initialPose.request.desiredPose.x = current_x;
    initialPose.request.desiredPose.y = current_y;
    initialPose.request.desiredPose.theta = current_angle;
    setPoseClient.call(initialPose);
  }

  // if(setPoseFake.exists())
  // {

  //   ROS_INFO_STREAM("Entered service client");
  //   rigid2d::SetPose initialPose;
  //   initialPose.request.desiredPose.x = current_x;
  //   initialPose.request.desiredPose.y = current_y;
  //   initialPose.request.desiredPose.theta = current_angle;
  //   setPoseFake.call(initialPose);
  // }
  state = WAIT;


  geometry_msgs::Twist velocity_message;
  velocity_publisher.publish(velocity_message);
  }
  /// \brief This function changes waypoint so that the robot can follow the next 
  ///        waypoint

  void change_way_point()
  {
    wayPointIndex += 1;
    if(isAllPointsVisited)
    {
      state = STOP;
    }
    else if(wayPointIndex == trajPoints.size())
    {
      wayPointIndex = 0;
      isAllPointsVisited = true;
      state = FOLLOW_WAYPOINT;
    }
    else
    {
      state = FOLLOW_WAYPOINT;
    }
    current_waypoint = trajPoints[wayPointIndex];
  }



  /// \brief This function contains the main state machine that is executed on every iteration.
  ///        It links each state to its corresponding higher level apis.
  bool execute_trajectory()
    {
     ROS_INFO_STREAM("state"<<state<<wayPointIndex);
      switch(state)
      {
      case INITIALISE       :initialise_settings();
                             break;
      case WAIT             :break;
      case FOLLOW_WAYPOINT :follow_way_point();
                            break;
      case CHANGE_WAY_POINT:change_way_point();
                            break;
      case STOP             :return false;
      }
      return true;
    }
  };

/// \brief The main function
int main(int argc, char** argv)
{
  WaypointTurtle turtlebotTrajectory(argc, argv);
  ros::Rate loop_rate(turtlebotTrajectory.frequency);
  bool shouldNodeRun = true;

  while (ros::ok() && shouldNodeRun)
  {
  shouldNodeRun = turtlebotTrajectory.execute_trajectory();
  loop_rate.sleep();
  ros::spinOnce();
  }
  return 0;

}

