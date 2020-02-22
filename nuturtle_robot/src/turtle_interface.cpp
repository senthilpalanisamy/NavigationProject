/// \file
/// \brief This file implements a node that interfaces the planning commands with the
/// the command inputs of turtlebot.
///
/// PARAMETERS:
/// ~frac_vel(double) : Fraction of the maximum velocity to use
///  max_rot_vel_robot(double) : Maximum rotation velocity of the robot
///
/// PUBLISHES:
/// wheel_cmd - (nuturtlebot/WheelCommands) - These wheel commands dictate the velocity
///                                           of the wheel motors
/// joint_states - (sensor_msgs/JointState) - The joint states of all robot joints are
///                                           published in this topic
/// SUBSCRIBES:
/// cmd_vel - (geometry_msgs/Twist) - Body Twist of the robot desired is published in this
///                                   topic
/// joint_states - (nuturtlebot/sensorData) - Data of all sensors in turtlebot are published
///                                           in this topic

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "nuturtlebot/WheelCommands.h" 
#include "nuturtlebot/SensorData.h" 
#include <cmath>
/// \brief This function calculates the clockwise distance between two angles
/// \param x - Angle 1 in radians
/// \param y - Angle 2 in radians

double clockwiseDistance(double x, double y)
{
  if(y > x)
  {
    return y-x;
   }
  else
  {
    return 2 * rigid2d::PI -x + y;
  }

}


/// \brief This function calculates the anti-clockwise distance between two angles
/// \param x - Angle 1 in radians
/// \param y - Angle 2 in radians

double anticlockwiseDistance(double x, double y)
{
  if(y < x)
  {
    return y-x;
   }
  else
  {
    return y - 2 * rigid2d::PI -x;
  }

}

/// \brief This function clamps the values of the given input between two extreme values
/// \param input - Input value which should be clamped between two extreme values
/// \param upperLimit - UpperLimit for clamping
/// \param lowerLimit - LowerLimit for clamping

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
  sensor_msgs::JointState jointStateMsg;

  std::vector<std::string> jointNames;
  std::vector<double> jointPositions;
  std::vector<double> jointVelocities;
  
  public:
  /// \brief A parameterized constructor for creating a turtle interface node
  /// \param argc - Number of command line arguments
  /// \param argv - Command line arguments
  turtleInterface(int argc, char **argv)
  {


  ros::init(argc, argv, "turtle_interface");
  ros::NodeHandle n;
  ros::Rate r(100.0);

  cmdVelSubscriber = n.subscribe("/cmd_vel", 1000, &turtleInterface::cmdVelCallback,
                                  this);

  sensorDataSubscriber = n.subscribe("/sensor_data", 10000, &turtleInterface::sensorDataCallback,
                                  this);
  wheelCommandPublisher = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 1000, true);
  jspPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000, true);

  ros::param::get("max_rot_vel_motor", maxMotorVelocity);
  ros::param::get("max_trans_vel", robotMaxTransVel);
  ros::param::get("max_rot_vel_robot", robotMaxRotVel);
  ros::param::get("wheel_base", wheelBase);
  ros::param::get("wheel_radius", wheelRadius);
  ros::param::get("~left_wheel_joint", leftWheelJoint);
  ros::param::get("~right_wheel_joint", rightWheelJoint);
  ros::param::get("encoder_ticks_per_rev", encoderTicksPerRevolution);
  ros::param::get("max_wheel_command", maxWheelCommand);

  rigid2d::Transform2D identityTransform(0);
  diffcar = rigid2d::DiffDrive(identityTransform, wheelBase, wheelRadius);
  scaleMotorVelToCmd = maxWheelCommand /  maxMotorVelocity;
  bIsFirstRun = true;
  jointStateMsg.header.frame_id = "turtle_interface";
  }

  /// \brief A call function for cmd_vel topic. This function converts the body twist into
  /// wheel commands for the turtle bot
  /// \param bodyTwistMsg - A geometry twist message containing the body twist of the robot.

  void cmdVelCallback(const geometry_msgs::Twist bodyTwistMsg)
    {
  
      ROS_INFO_STREAM("linear velocity before clamp"<<bodyTwistMsg.linear.x);
      ROS_INFO_STREAM("angular velocity before clamp"<<bodyTwistMsg.angular.z);
      ROS_INFO_STREAM("robot Max Trans Velocity"<<robotMaxTransVel);
      ROS_INFO_STREAM("robot Max Rotational Velocity"<<robotMaxRotVel);
      auto linearVelocity = clampValue(bodyTwistMsg.linear.x, robotMaxTransVel, -robotMaxTransVel);
      auto angularVelocity = clampValue(bodyTwistMsg.angular.z, robotMaxRotVel, -robotMaxRotVel);
      ROS_INFO_STREAM("linear velocity"<<linearVelocity);
      ROS_INFO_STREAM("angular velocity"<<angularVelocity);
  
      rigid2d::Twist2D twistFollowed;
      twistFollowed.wz = bodyTwistMsg.angular.z;
      twistFollowed.vx = bodyTwistMsg.linear.x;
      twistFollowed.vy = bodyTwistMsg.linear.y;
  
      auto velocities = diffcar.twistToWheelVelocities(twistFollowed);
      ROS_INFO_STREAM("left velocity"<<velocities.left);
      ROS_INFO_STREAM("right velocity"<<velocities.right);
      velocities.left = clampValue(velocities.left, maxMotorVelocity, -maxMotorVelocity);
      velocities.right = clampValue(velocities.right, maxMotorVelocity, -maxMotorVelocity);
      ROS_INFO_STREAM("clamped left velocity"<<velocities.left);
      ROS_INFO_STREAM("clamped right command"<<velocities.right);
      nuturtlebot::WheelCommands wheelVelCommands;
      wheelVelCommands.left_velocity = round(velocities.left * scaleMotorVelToCmd);
      wheelVelCommands.right_velocity = round(velocities.right * scaleMotorVelToCmd);
      ROS_INFO_STREAM("scaled left velocity"<<wheelVelCommands.left_velocity);
      ROS_INFO_STREAM("scaled right command"<<wheelVelCommands.right_velocity);
      wheelCommandPublisher.publish(wheelVelCommands);
  }

  /// \brief This function calculates the wheel velocities of the robot based on 
  ///        the previous wheel position, the present wheel position and the time
  ///        between the two measurements.
  /// \param previousPoint - The previous position of the wheel
  /// \param presentPoint  - current position of the wheel
  /// \param time          - The time between the two measurements
  double calculateWheelVelocities(double previousPoint, double presentPoint, double time)
  {

      double wheelVelocity = (presentPoint - previousPoint) / time;
      return wheelVelocity;

  }
  /// \brief This function normalizes the encoder position values i.e, converts the raw
  ///        encoder ticks to distance travelled in terms of (wheel) radians
  /// \param encoderValue - Number of ticks recorded by the encoder.

  double normaliseEncoderValues(const int& encoderValue)
  {

     double normalisedRotCount = double(encoderValue) / double(encoderTicksPerRevolution) *
                                 rigid2d::PI*2;
     return normalisedRotCount;

  }
  /// \brief A call back function for sensordata messages published. This function reads
  ///        sensor data and calculates the joint states of the robot and publishes it
  ///        in the joint states topic
  /// \param turtlebotSensor - Message containing all sensor messages of the turtlebot.

  void sensorDataCallback(const nuturtlebot::SensorData turtlebotSensor)
  {

      currentTime = turtlebotSensor.stamp;
      jointStateMsg.header.stamp = currentTime;

      jointNames = {leftWheelJoint, rightWheelJoint};
      int leftEncoderTicks = turtlebotSensor.left_encoder;
      int rightEncoderTicks = turtlebotSensor.right_encoder;


      jointPositions = {normaliseEncoderValues(leftEncoderTicks), normaliseEncoderValues(rightEncoderTicks)};

      if(bIsFirstRun)
      {
        bIsFirstRun = false;
        jointVelocities = {0.0, 0.0};
      }
      else
      {

      ros::Duration totalDuration = currentTime - lastTime;
      double totalTime = totalDuration.toSec();
      double leftWheelVelocity = calculateWheelVelocities(leftPrevPosition, jointPositions[0], totalTime);
      double rightWheelVelocity = calculateWheelVelocities(rightPrevPosition, jointPositions[1], totalTime);
      jointVelocities = {leftWheelVelocity, rightWheelVelocity};
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
