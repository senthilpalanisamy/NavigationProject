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
#include <math.h>

bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
{
   return(abs(d1-d2) < epsilon? true: false);
}


double clockwiseDistance(double x, double y)
{
  if(y >= x)
  {
    return y-x;
   }
  else
  {
    return 2 * rigid2d::PI -x + y;
  }

}


double anticlockwiseDistance(double x, double y)
{
  if(y <= x)
  {
    return y-x;
   }
  else
  {
    return y - 2 * rigid2d::PI -x;
  }

}



double calculateWheelVelocities(double previousPoint, double presentPoint, double time)
{

    double clkDistance = clockwiseDistance(previousPoint, presentPoint);
    double anticlkDistance = anticlockwiseDistance(previousPoint, presentPoint);

    // ROS_INFO_STREAM("clockwise_distance"<<clkDistance);
    // ROS_INFO_STREAM("anticlockwise_distance"<<anticlkDistance);
    double wheelVelocity;
    if(abs(anticlkDistance) < clkDistance)
    {
     wheelVelocity = anticlkDistance / time;
    }
    else
    {
      wheelVelocity = clkDistance/ time;
    }
    return wheelVelocity;

}



namespace odometry
{

odometry::odometry::odometry(int argc, char** argv)
{

   ros::init(argc, argv, "odometer");
   ros::NodeHandle n;
   tf::TransformBroadcaster odom_broadcaster;
   currentTime = ros::Time::now();
   lastTime = ros::Time::now();
   ros::Rate r(100);
   ros::param::get("/wheel_base", wheelBase);
   ros::param::get("/wheel_radius", wheelRadius);
   nameSpace = ros::this_node::getNamespace();
   jointStataSubscriber = n.subscribe("joint_states", 1000, &odometry::jointStatesCallback,
                                    this);
   odometryPublisher = n.advertise<nav_msgs::Odometry>("nav_msgs/odometry", 1000);
   ros::param::get("~odom_frame_id", odom_frame_id);
   ros::param::get("~body_frame_id", body_frame_id);
   ros::param::get("~left_wheel_joint", left_wheel_joint);
   ros::param::get("~right_wheel_joint", right_wheel_joint);
   if(nameSpace != "/")
   {
     odom_frame_id = nameSpace +"_" + odom_frame_id;
     body_frame_id = nameSpace +"_" + body_frame_id;
   }

  ROS_INFO_STREAM("namespace:"<<nameSpace);
  ROS_INFO_STREAM("namespace"<<ros::this_node::getNamespace());
  ROS_INFO_STREAM("odom_frame_id"<<odom_frame_id);
 ROS_INFO_STREAM("body_frame_id"<<body_frame_id);
 ROS_INFO_STREAM("wheel_base"<<wheelBase);
 ROS_INFO_STREAM("wheel_radius"<<wheelRadius);

   lastTime = ros::Time::now();
   leftWheelPosition = 0.0;
   rightWheelPosition = 0.0;
   previousLeftPosition = 0.0;
   previousRightPosition = 0.0;
   rigid2d::Transform2D identityTransform(0);
   diffcar = rigid2d::DiffDrive(identityTransform, wheelBase, wheelRadius); 
   setTurtlePose = n.advertiseService("set_pose", &odometry::setTurtlePoseCallback,
                                     this);
  bIsFirstRun = true; 
}

bool odometry::setTurtlePoseCallback(rigid2d::SetPose::Request& request,
                                     rigid2d::SetPose::Response& response)
{


   rigid2d::Vector2D newPosition = {request.desiredPose.x, request.desiredPose.y};
   rigid2d::Transform2D newPose(newPosition, request.desiredPose.theta);
   rigid2d::DiffDrive diffcarNewPose(newPose, wheelBase, wheelRadius);
   diffcar = diffcarNewPose;
   ROS_INFO_STREAM("Inside service\n");
   return true;
}


void odometry::jointStatesCallback(const sensor_msgs::JointState jointMessage)
{

  currentTime = jointMessage.header.stamp;
  double newLeftPosition = jointMessage.position[0];
  double newRightPosition = jointMessage.position[1];
  ROS_INFO_STREAM("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  //ROS_INFO_STREAM("left position"<<leftDistance);
  //ROS_INFO_STREAM("right position"<<rightDistance);


  auto carPose = diffcar.returnPose();
  rigid2d::Twist2D bodyTwist;

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
  ROS_INFO_STREAM("previousLeftPosition"<<previousLeftPosition);
  ROS_INFO_STREAM("previousRightPosition"<<previousRightPosition);
  ROS_INFO_STREAM("newLeftPosition"<<newLeftPosition);
  ROS_INFO_STREAM("newRightPosition"<<newRightPosition);
  ROS_INFO_STREAM("time_duration"<<time_duration);

  //double leftVelocity = calculateWheelVelocities(previousLeftPosition, newLeftPosition, totalTime);
  //double rightVelocity = calculateWheelVelocities(previousRightPosition, newRightPosition, totalTime);
  double leftVelocity = jointMessage.velocity[0];
  double rightVelocity = jointMessage.velocity[1];

  ROS_INFO_STREAM("left velocity"<<leftVelocity * totalTime);
  ROS_INFO_STREAM("right velocity"<<rightVelocity * totalTime);
  rigid2d::WheelVelocities velocities = {leftVelocity, rightVelocity};
  bodyTwist = diffcar.WheelVelocitiestoTwist(velocities);
  ROS_INFO_STREAM("Body Twist z x y"<<bodyTwist.wz<<" "<<bodyTwist.vx<<" "<<bodyTwist.vy);
  ROS_INFO_STREAM("time taken:"<<totalTime);

  carPose = diffcar.returnPose();

  ROS_INFO_STREAM("before car pose x"<<carPose.x);
  ROS_INFO_STREAM("before car pose y"<<carPose.y);
  ROS_INFO_STREAM("before car pose theta"<<carPose.theta);

  diffcar.UpdateOdometry(leftVelocity * totalTime, rightVelocity * totalTime);
  carPose = diffcar.returnPose();

  ROS_INFO_STREAM("after car pose x"<<carPose.x);
  ROS_INFO_STREAM("after car pose y"<<carPose.y);
  ROS_INFO_STREAM("after car pose theta"<<carPose.theta);

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

  ROS_INFO_STREAM("car pose x"<<carPose.x);
  ROS_INFO_STREAM("car pose y"<<carPose.y);
  ROS_INFO_STREAM("car pose theta"<<carPose.theta);
  ROS_INFO_STREAM("presentPosition left"<<jointMessage.position[0]);
  ROS_INFO_STREAM("presentPosition right"<<jointMessage.position[1]);

  //static tf2_ros::TransformBroadcaster br;
  static tf::TransformBroadcaster br;
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


  lastTime = jointMessage.header.stamp ;

  ROS_INFO_STREAM("\n Finished");
  ROS_INFO_STREAM("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  previousLeftPosition = jointMessage.position[0];
  previousRightPosition = jointMessage.position[1];


}

}

int main(int argc, char** argv)
{

  odometry::odometry odomNode(argc, argv);
  ros::spin();
  return 0;
}



