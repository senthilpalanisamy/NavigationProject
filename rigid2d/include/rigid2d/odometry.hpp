#ifndef ODOMETRY_INCLUDE_GUARD_HPP
#define ODOMETRY_INCLUDE_GUARD_HPP
/// \brief A node that tracks and publishes the odometry of the robot
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include "rigid2d/SetPose.h"


using namespace std;
namespace odometry
{
/// \brief A class which tracks, maintains and publishes the odometry of the 
/// robot
class odometry
{
  public:
    /// \brief parameterised constructor
    /// \param argc - arguments from main to initialize the node
    /// param arv - argument from main to initialize the node
    odometry(int argc, char **argv);
  private:
    /// \brief callback function for joint states. Once joint states are received
    /// the odometry is updated and then published in the navigation topic
    void jointStatesCallback(const sensor_msgs::JointState);
    /// frame names
    string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
    /// Subscriber and publisher for communication
    ros::Subscriber jointStataSubscriber;
    ros::Publisher odometryPublisher;
    ros::ServiceServer setTurtlePose;
    /// other private parameters to keep track of the states and necessary
    /// information about the robot
    double wheelBase, wheelRadius, previousLeftPosition, previousRightPosition;
    rigid2d::DiffDrive diffcar;
    float leftWheelPosition, rightWheelPosition;
    ros::Time currentTime, lastTime;
    bool bIsFirstRun;
    bool setTurtlePoseCallback(rigid2d::SetPose::Request& request,
                               rigid2d::SetPose::Response& response);
    string nameSpace;
};
}
#endif
