#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "nuturtlebot/WheelCommands.h" 
#include "nuturtlebot/SensorData.h" 

double clampValue(double input, double upperLimit, double lowerLimit)
{
  double clampedValue;

  if(input > 0)
    {
      clampedValue = std::min(input, upperLimit);
    }
  else
    {
      clampedValue = std::max(input, lowerLimit);
    }
  return clampedValue;

}

class turtleInterface
{

  ros::Subscriber cmdVelSubscriber, sensorDataSubscriber;
  ros::Publisher wheelCommandPublisher, jspPublisher;
  double maxMotorVelocity, robotMaxTransVel, robotMaxRotVel;
  double wheelBase, wheelRadius;
  rigid2d::DiffDrive diffcar;
  double maxWheelCommand, minWheelCommand, scaleMotorVelToCmd;
  std::string leftWheelJoint, rightWheelJoint;
  bool bIsFirstRun;
  int encoderTicksPerRevolution;
  float leftPrevPosition, rightPrevPosition;
  ros::Time currentTime, lastTime;
  
  public:
  turtleInterface(int argc, char **argv)
  {


  ros::init(argc, argv, "turtle_interface");
  ros::NodeHandle n;

  cmdVelSubscriber = n.subscribe("/cmd_vel", 1000, &turtleInterface::cmdVelCallback,
                                  this);

  sensorDataSubscriber = n.subscribe("/sensor_data", 1000, &turtleInterface::sensorDataCallback,
                                  this);
  wheelCommandPublisher = n.advertise<nuturtlebot::WheelCommands>("/wheel_commands", 1000);
  //jspPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);
  jspPublisher = n.advertise<sensor_msgs::JointState>("/jsp", 1000);

  ros::param::get("max_rot_vel_motor", maxMotorVelocity);
  ros::param::get("max_trans_vel", robotMaxTransVel);
  ros::param::get("max_rot_vel_motor", robotMaxRotVel);
  ros::param::get("wheel_base", wheelBase);
  ros::param::get("wheel_radius", wheelRadius);
  ros::param::get("~left_wheel_joint", leftWheelJoint);
  ros::param::get("~right_wheel_joint", rightWheelJoint);
  ros::param::get("encoder_ticks_per_rev", encoderTicksPerRevolution);


  rigid2d::Transform2D identityTransform(0);
  diffcar = rigid2d::DiffDrive(identityTransform, wheelBase, wheelRadius); 
  maxWheelCommand = 44;
  minWheelCommand = -44;
  scaleMotorVelToCmd = (maxWheelCommand - minWheelCommand) / (2  * maxMotorVelocity);
  bIsFirstRun = true;
  }

void cmdVelCallback(const geometry_msgs::Twist bodyTwistMsg)

  {
    auto linearVelocity = clampValue(bodyTwistMsg.linear.x, robotMaxTransVel, -robotMaxTransVel);
    auto angularVelocity = clampValue(bodyTwistMsg.angular.z, robotMaxRotVel, -robotMaxRotVel);

    rigid2d::Twist2D twistFollowed;
    twistFollowed.wz = bodyTwistMsg.angular.z;
    twistFollowed.vx = bodyTwistMsg.linear.x;
    twistFollowed.vy = bodyTwistMsg.linear.y;

    auto velocities = diffcar.twistToWheelVelocities(twistFollowed);
    velocities.left = clampValue(velocities.left, maxMotorVelocity, -maxMotorVelocity);
    velocities.right = clampValue(velocities.right, maxMotorVelocity, -maxMotorVelocity);
    nuturtlebot::WheelCommands wheelVelCommands;
    wheelVelCommands.left_velocity = velocities.left * scaleMotorVelToCmd;
    wheelVelCommands.right_velocity = velocities.right * scaleMotorVelToCmd;
    wheelCommandPublisher.publish(wheelVelCommands);
}

void sensorDataCallback(const nuturtlebot::SensorData turtlebotSensor)
{

    currentTime = ros::Time::now();
    sensor_msgs::JointState jointStateMsg;
    jointStateMsg.header.stamp = currentTime;
    jointStateMsg.header.frame_id = "turtle_interface";



    std::vector<std::string> jointNames = {leftWheelJoint, rightWheelJoint};
    std::vector<double> jointPositions;
    std::vector<double> jointVelocities;



    if(bIsFirstRun)
    {
      jointPositions = {0.0, 0.0};
      bIsFirstRun = false;
    }

    jointPositions = {turtlebotSensor.left_encoder / float(encoderTicksPerRevolution),
                     turtlebotSensor.right_encoder / float(encoderTicksPerRevolution)};
    if(bIsFirstRun)
    {
      jointVelocities = {0.0, 0.0};
    }
    else
    {

    ros::Duration totalDuration = currentTime - lastTime;
    double totalTime = totalDuration.toSec();
    jointVelocities = {(jointPositions[0] - leftPrevPosition) / totalTime,
                     (jointPositions[0] - rightPrevPosition) / totalTime};
    }



    jointStateMsg.name = jointNames;
    jointStateMsg.position = jointPositions;
    jointStateMsg.velocity = jointVelocities;
    jspPublisher.publish(jointStateMsg);

    lastTime = currentTime;
    leftPrevPosition = jointPositions[0];
    rightPrevPosition = jointPositions[1];

}
};


int main(int argc, char** argv )
{
  turtleInterface turtleBot(argc, argv);
  ros::spin();
  return 0;
}
