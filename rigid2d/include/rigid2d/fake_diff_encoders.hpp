#ifndef FAKE_ENCODERS_INCLUDE_GUARD_HPP
#define FAKE_ENCODERS_INCLUDE_GUARD_HPP
/// \file
/// \brief A node implementing a fake encoder. The encoders simulates
/// a encoder on the robot and gives out encoder readings
#include <ros/ros.h>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Twist.h>

namespace FakeEncoder
{
  /// \brief A class for publishing fake odometry messages 
  class FakeEncoder
  {
    public:
    /// \brief A parameterized constructor for initializing the node and other
    /// startup parameters
    FakeEncoder(int argc, char** argv);
    private:
    /// \brief The call back function for cmd_vel topic. It reads the twist
    /// the robot needs to follow and computes the corresponding wheel movement
    /// and publishes the fake odometry message
    /// \param bodyTwistMsg - The twist message published by cmd_vel topic
    void cmdVelCallback(geometry_msgs::Twist bodyTwistMsg);
    /// Subscriber and publisher used in the program
    ros::Subscriber cmdVelSubscriber;
    ros::Publisher jointStatePublisher;
    /// Other parameters useful for maintaining states and information about
    /// the system
    double wheelRadius, wheelBase, previousLeftPosition, previousRightPosition;
    std::string leftWheelJoint, rightWheelJoint;
    ros::Time currentTime, lastTime;
    geometry_msgs::Twist previousTwistMsg;
    rigid2d::DiffDrive diffcar;
    bool bIsFirstRun;

  };
}
#endif
