#ifndef FAKE_ENCODERS_INCLUDE_GUARD_HPP
#define FAKE_ENCODERS_INCLUDE_GUARD_HPP
#include <ros/ros.h>
#include <string>
#include "rigid2d/rigid2d.hpp"
#include <geometry_msgs/Twist.h>

namespace FakeEncoder
{
  class FakeEncoder
  {
    public:
    FakeEncoder(int argc, char** argv);
    private:
    void cmdVelCallback(geometry_msgs::Twist bodyTwistMsg);
    ros::Subscriber cmdVelSubscriber;
    ros::Publisher jointStatePublisher;
    double wheelRadius, wheelBase;
    std::string leftWheelJoint, rightWheelJoint;
    ros::Time currentTime, lastTime;
    geometry_msgs::Twist previousTwistMsg;
    rigid2d::DiffDrive diffcar;
    bool bIsFirstRun;

  };
}
#endif
