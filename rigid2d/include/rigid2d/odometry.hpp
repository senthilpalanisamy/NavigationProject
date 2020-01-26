#ifndef ODOMETRY_INCLUDE_GUARD_HPP
#define ODOMETRY_INCLUDE_GUARD_HPP

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>



using namespace std;
namespace odometry
{
class odometry
{
  public:
    odometry(int argc, char **argv);
  private:
    string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
    ros::Subscriber jointStataSubscriber;
    ros::Publisher odometryPublisher;
    double wheelBase, wheelRadius;
    void jointStatesCallback(const sensor_msgs::JointState);
    rigid2d::DiffDrive diffcar;
    float leftWheelPosition, rightWheelPosition;
    ros::Time currentTime, lastTime;
    ros::Publisher odom_pub;

};
}
#endif
