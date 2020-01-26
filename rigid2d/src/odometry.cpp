#include "rigid2d/odometry.hpp"
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include<ros/console.h>


namespace odometry
{
odometry::odometry::odometry(int argc, char** argv)
{
   ros::init(argc, argv, "odometry_publisher");
   ros::NodeHandle n;
   //odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
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
   // left_wheel_joint = "left";
   // right_wheel_joint = "right";
   lastTime = ros::Time::now();
   leftWheelPosition = 0.0;
   rightWheelPosition = 0.0;
   ROS_INFO_STREAM("\n finished initialising");
                  
}


void odometry::jointStatesCallback(const sensor_msgs::JointState jointMessage)
{

   ROS_INFO_STREAM("\n Inside subscriber");

  currentTime = ros::Time::now();
  float leftDistance = jointMessage.position[0] - leftWheelPosition;
  float rightDistance = jointMessage.velocity[1] - rightWheelPosition;
  diffcar.UpdateOdometry(leftDistance, rightDistance);

  auto carPose = diffcar.returnPose();
  ros::Duration time_duration = currentTime - lastTime;
  double totalTime = time_duration.toSec();
  rigid2d::WheelVelocities velocities = {leftDistance / totalTime, rightDistance / totalTime};
  auto bodyTwist = diffcar.WheelVelocitiestoTwist(velocities);

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

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped; 
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = odom_frame_id;
  transformStamped.child_frame_id = body_frame_id;
  transformStamped.transform.translation.x = carPose.x;
  transformStamped.transform.translation.y = carPose.y;
  transformStamped.transform.translation.z = 0.0;
  //tf2::Quaternion q;
  //q.setRPY(0, 0, carPose.theta);
  transformStamped.transform.rotation.x = odomQuat.x;
  transformStamped.transform.rotation.y = odomQuat.y;
  transformStamped.transform.rotation.z = odomQuat.z;
  transformStamped.transform.rotation.w = odomQuat.w;
  br.sendTransform(transformStamped);
  lastTime = currentTime;


}

}

int main(int argc, char** argv)
{

  odometry::odometry odomNode(argc, argv);
  //ros::Rate loop_rate(10.0);

  ros::spin();
  return 0;
}


