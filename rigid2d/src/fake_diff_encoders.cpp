#include"rigid2d/fake_diff_encoders.hpp"
#include <sensor_msgs/JointState.h>

namespace FakeEncoder
{
  FakeEncoder::FakeEncoder(int argc, char** argv)
  {

  ros::init(argc, argv, "fake_diff_encoders");
  ros::NodeHandle n;
  ros::Rate r(10.0);
  cmdVelSubscriber = n.subscribe("cmd_vel", 1000, &FakeEncoder::cmdVelCallback,
                                    this);
  jointStatePublisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::param::get("wheel_base", wheelBase);
  ros::param::get("wheel_radius", wheelRadius);
  ros::param::get("left_wheel_joint", leftWheelJoint);
  ros::param::get("right_wheel_joint", rightWheelJoint);
  currentTime = ros::Time::now();
  lastTime = ros::Time::now();
  bIsFirstRun = true;
  }

  void FakeEncoder::cmdVelCallback(const geometry_msgs::Twist bodyTwistMsg)
  {
    sensor_msgs::JointState jointStateMsg;
    currentTime = ros::Time::now();
    std::vector<std::string> jointNames = {"left", "right"};
    std::vector<double> jointPosition;

    if(bIsFirstRun)
    {
      lastTime = ros::Time::now();
      previousTwistMsg = bodyTwistMsg;
      jointPosition = {0.0, 0.0};
      bIsFirstRun = false;
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
    jointPosition = {velocities.left * totalTime, velocities.right * totalTime};
    }

    jointStateMsg.name = jointNames;
    jointStateMsg.position = jointPosition;
    jointStatePublisher.publish(jointStateMsg);
  }

}


int main(int argc, char** argv)
{
  FakeEncoder::FakeEncoder diffEncoder(argc, argv);
  ros::spin();
  return 0;
}
