/// \file
/// \brief This file contains all functions definitions for the fake encoder node 
/// This node contains all functionalities to simulate a fake encoder for a robot navigation 
///
/// PARAMETERS:
///  wheel_base   (double) : Wheel to wheel distance of the differential drive robot
///  wheel_radius (double) : Wheel radius of the differential drive robot
///  ~left_wheel_joint (string) : Name of the left wheel joint 
///  ~right_wheel_joint(string) : Name of the right wheel joint
/// PUBLISHES:
///  joint_states (sensor_msgs/JointState) : A topic for publishing the joint state of various
///                                           joints on the robot
///
/// SUBSCRIBES:
///  cmd_vel (geometry_msgs/Twist) : A topic where the twist the robot needs to follow is
///                                  published
#include"rigid2d/fake_diff_encoders.hpp"
#include <sensor_msgs/JointState.h>
#include <ros/console.h>

double wrapAngle0to2Pi(double angle)
{
  double cTheta = cos(angle);
  double sTheta = sin(angle);
  double theta = atan2(sTheta, cTheta);
  if(theta < 0)
  {
    theta = 2 * rigid2d::PI - abs(theta);
  }
  return theta;

}

namespace FakeEncoder
{
  FakeEncoder::FakeEncoder(int argc, char** argv)
  {

  ros::init(argc, argv, "fake_diff_encoders");
  ros::NodeHandle n;
  ros::Rate r(100.0);
  cmdVelSubscriber = n.subscribe("cmd_vel", 1000, &FakeEncoder::cmdVelCallback,
                                    this);
  jointStatePublisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::param::get("wheel_base", wheelBase);
  ros::param::get("wheel_radius", wheelRadius);
  ros::param::get("~left_wheel_joint", leftWheelJoint);
  ros::param::get("~right_wheel_joint", rightWheelJoint);
  ROS_INFO_STREAM("got all params"<<leftWheelJoint<<rightWheelJoint<<"\n");
  currentTime = ros::Time::now();
  lastTime = ros::Time::now();
  bIsFirstRun = true;
  rigid2d::Transform2D identityTransform(0);
  diffcar = rigid2d::DiffDrive(identityTransform, wheelBase, wheelRadius); 
  previousLeftPosition = 0.0;
  previousRightPosition = 0.0;
  }

  void FakeEncoder::cmdVelCallback(const geometry_msgs::Twist bodyTwistMsg)
  {
    currentTime = ros::Time::now();
    sensor_msgs::JointState jointStateMsg;

    jointStateMsg.header.stamp = currentTime;
    jointStateMsg.header.frame_id = "diff_drive";
    std::vector<std::string> jointNames = {leftWheelJoint, rightWheelJoint};
    std::vector<double> jointPosition;
    std::vector<double> jointVelocities;
    jointVelocities = {0.0, 0.0};

    if(bIsFirstRun)
    {
      previousTwistMsg = bodyTwistMsg;
      jointPosition = {0.0, 0.0};
      bIsFirstRun = false;
      previousLeftPosition = 0.0;
      previousRightPosition = 0.0;
    }
    else
    {

    ros::Duration time_duration = currentTime - lastTime;
    double totalTime = time_duration.toSec();
    rigid2d::Twist2D twistFollowed;
    twistFollowed.wz = previousTwistMsg.angular.z;
    twistFollowed.vx = previousTwistMsg.linear.x;
    twistFollowed.vy = previousTwistMsg.linear.y;
    auto velocities = diffcar.twistToWheelVelocities(twistFollowed, totalTime);
    double leftMovement =  velocities.left * totalTime;
    double rightMovement = velocities.right * totalTime;
    jointPosition = {previousLeftPosition + leftMovement,
                    previousRightPosition + rightMovement};
    jointVelocities = {(jointPosition[0] - previousLeftPosition) / totalTime,
                       (jointPosition[1] - previousRightPosition) / totalTime};

    previousLeftPosition = jointPosition[0];
    previousRightPosition = jointPosition[1];
    //previousLeftPosition =  wrapAngle0to2Pi(previousLeftPosition);
    //previousRightPosition = wrapAngle0to2Pi(previousRightPosition);
    previousTwistMsg = bodyTwistMsg;
    }

    jointStateMsg.name = jointNames;
    jointStateMsg.position = jointPosition;
    jointStateMsg.velocity = jointVelocities;
    jointStatePublisher.publish(jointStateMsg);
    lastTime = currentTime;
  }

}


int main(int argc, char** argv)
{
  FakeEncoder::FakeEncoder diffEncoder(argc, argv);
  ros::spin();
  return 0;
}
