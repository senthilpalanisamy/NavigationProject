/// \file
/// \brief This file contains all functions definitions for the updating the odometry of 
/// a differential drive robot based on the wheel velocities received.
/// PARAMETERS:
///  ~odom_frame_id (string)    - Name of the odometry frame
///  ~body_frame_id (string)    - Name of the baselink of the robot
///  ~left_wheel_joint (sring)  - Name of the left wheel joint
///  ~right_wheel_joint (string)- Name of the right wheel joint
///  wheel_base       (double)  - wheel to wheel distance
///  wheel_radius     (double)  - wheel radius
/// PUBLISHES:
///   /nav_msgs/odometry (nav_msgs/odometry) - The odometry of the robot is published in 
///                                            this topic. It published every time a message
///                                            is received from joint_states
/// SUBSCRIBES:
///  joint_states (sensor_msgs/JointState)  - A topic where the position, velocity, torque
///                                           and all other states of the robot joints is 
///                                           published
#include "rigid2d/odometry.hpp"
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/console.h>


namespace odometry
{
odometry::odometry::odometry(int argc, char** argv)
{
   ros::init(argc, argv, "odometer");
   ros::NodeHandle n;
   tf::TransformBroadcaster odom_broadcaster;
   currentTime = ros::Time::now();
   lastTime = ros::Time::now();
   ros::Rate r(10.0);
   ros::param::get("wheel_base", wheelBase);
   ros::param::get("wheel_radius", wheelRadius);
   jointStataSubscriber = n.subscribe("joint_states", 1000, &odometry::jointStatesCallback,
                                    this);
   odometryPublisher = n.advertise<nav_msgs::Odometry>("/nav_msgs/odometry", 1000);
   ros::param::get("~odom_frame_id", odom_frame_id);
   ros::param::get("~body_frame_id", body_frame_id);
   ros::param::get("~left_wheel_joint", left_wheel_joint);
   ros::param::get("~right_wheel_joint", right_wheel_joint);
   lastTime = ros::Time::now();
   leftWheelPosition = 0.0;
   rightWheelPosition = 0.0;
   rigid2d::Transform2D identityTransform(0);
   diffcar = rigid2d::DiffDrive(identityTransform, wheelBase, wheelRadius); 
}


void odometry::jointStatesCallback(const sensor_msgs::JointState jointMessage)
{

  currentTime = ros::Time::now();
  double leftDistance = jointMessage.position[0];
  double rightDistance = jointMessage.position[1];


  auto carPose = diffcar.returnPose();
  diffcar.UpdateOdometry(leftDistance, rightDistance);
  rigid2d::Twist2D bodyTwist;
  carPose = diffcar.returnPose();


  if(bIsFirstRun)
  {
    lastTime = currentTime;
    bodyTwist = {0.0, 0.0, 0.0};
    bIsFirstRun = false;

  }
  else
  {
  ros::Duration time_duration = currentTime - lastTime;
  double totalTime = time_duration.toSec();
  rigid2d::WheelVelocities velocities = {leftDistance / totalTime, rightDistance / totalTime};
  bodyTwist = diffcar.WheelVelocitiestoTwist(velocities);
  }

  ROS_INFO_STREAM("\n Publishing message");

  nav_msgs::Odometry odom;
  odom.header.stamp = currentTime;
  odom.header.frame_id = odom_frame_id;
  geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(carPose.theta);

  //set the position
  odom.pose.pose.position.x = carPose.x;
  odom.pose.pose.position.y = carPose.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odomQuat;


  //set the velocity
  odom.child_frame_id = body_frame_id;
  odom.twist.twist.linear.x = bodyTwist.vx;
  odom.twist.twist.linear.y = bodyTwist.vy;
  odom.twist.twist.angular.z = bodyTwist.wz;

  //publish the message
  odometryPublisher.publish(odom);

  ROS_INFO_STREAM("\n Publishing transform");

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped; 
  transformStamped.header.stamp = currentTime;
  transformStamped.header.frame_id = odom_frame_id;
  transformStamped.child_frame_id = body_frame_id;
  transformStamped.transform.translation.x = carPose.x;
  transformStamped.transform.translation.y = carPose.y;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = odomQuat.x;
  transformStamped.transform.rotation.y = odomQuat.y;
  transformStamped.transform.rotation.z = odomQuat.z;
  transformStamped.transform.rotation.w = odomQuat.w;
  br.sendTransform(transformStamped);
  lastTime = currentTime;

  ROS_INFO_STREAM("\n Finished");
  leftWheelPosition = jointMessage.position[0];
  rightWheelPosition = jointMessage.position[1];


}

}

int main(int argc, char** argv)
{

  odometry::odometry odomNode(argc, argv);
  ros::spin();
  return 0;
}



