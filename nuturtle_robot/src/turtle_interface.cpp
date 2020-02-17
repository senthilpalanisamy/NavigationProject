#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "nuturtlebot/WheelCommands.h" 
#include "nuturtlebot/SensorData.h" 
#include <cmath>

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
  //jspPublisher = n.advertise<sensor_msgs::JointState>("/jsp", 1000);

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

double calculateWheelVelocities(double previousPoint, double presentPoint, double time)
{

    //double clkDistance = clockwiseDistance(previousPoint, presentPoint);
    //double anticlkDistance = anticlockwiseDistance(previousPoint, presentPoint);

    // ROS_INFO_STREAM("clockwise_distance"<<clkDistance);
    // ROS_INFO_STREAM("anticlockwise_distance"<<anticlkDistance);
    //double wheelVelocity;
    //if(abs(anticlkDistance) < clkDistance)
    //{
    // wheelVelocity = anticlkDistance / time;
    //}
    //else
    //{
    //  wheelVelocity = clkDistance/ time;
    //}
    double wheelVelocity = (presentPoint - previousPoint) / time;
    return wheelVelocity;

}

double normaliseEncoderValues(const int& encoderValue)
{

   int EncoderTicks;
   //double EncoderRevCount;
   //double normalisedRotCount;
   //if(encoderValue > 0)
   //{
   // EncoderTicks = encoderValue % encoderTicksPerRevolution;
   // EncoderRevCount = floor(encoderValue / double(encoderTicksPerRevolution)) * rigid2d::PI * 2;
   //}
   //else
   //{
   // EncoderTicks = abs(encoderValue) % encoderTicksPerRevolution;
   // EncoderTicks = - EncoderTicks;
   // EncoderRevCount = floor(abs(encoderValue) / double(encoderTicksPerRevolution)) * rigid2d::PI * 2
   //}
   //double jointState = EncoderTicks / double(encoderTicksPerRevolution) * rigid2d::PI * 2;
   double normalisedRotCount = double(encoderValue) / double(encoderTicksPerRevolution) *
                               rigid2d::PI*2;
   return normalisedRotCount;


}


void sensorDataCallback(const nuturtlebot::SensorData turtlebotSensor)
{

    currentTime = turtlebotSensor.stamp;
    jointStateMsg.header.stamp = currentTime;



    jointNames = {leftWheelJoint, rightWheelJoint};
    //std::vector<double> jointPositions;
    //std::vector<double> jointVelocities;

    int leftEncoderTicks = turtlebotSensor.left_encoder;
    int rightEncoderTicks = turtlebotSensor.right_encoder;


    //jointPositions = {leftEncoderTicks / double(encoderTicksPerRevolution) * rigid2d::PI * 2.0,
    //                 rightEncoderTicks / double(encoderTicksPerRevolution) * rigid2d::PI * 2.0};
    //ROS_INFO_STREAM("****************************************");
    //ROS_INFO_STREAM("left Wheel Positions before"<<leftEncoderTicks);
    //ROS_INFO_STREAM("right Wheel Positions before"<<rightEncoderTicks);
    jointPositions = {normaliseEncoderValues(leftEncoderTicks), normaliseEncoderValues(rightEncoderTicks)};
    //ROS_INFO_STREAM("right wheel Positions after"<<jointPositions[0]);
    //ROS_INFO_STREAM("right wheel Positions after"<<jointPositions[1]);

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

    //ROS_INFO_STREAM("right wheel velocity"<<leftWheelVelocity);
    //ROS_INFO_STREAM("right wheel velocity"<<rightWheelVelocity);
    //ROS_INFO_STREAM("****************************************");
    // ROS_INFO_STREAM("left wheel velocity"<<leftWheelVelocity);
    // ROS_INFO_STREAM("right wheel velocity"<<rightWheelVelocity);
    // ROS_INFO_STREAM("total time"<<totalDuration);
    // ROS_INFO_STREAM("right previous wheel position"<<rightPrevPosition);
    // ROS_INFO_STREAM("left previous wheel position"<<leftPrevPosition);
    // ROS_INFO_STREAM("right wheel position"<<jointPositions[1]);
    // ROS_INFO_STREAM("left  wheel position"<<jointPositions[0]);

    // ROS_INFO_STREAM("total time"<<totalDuration);

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
