#include <ros/ros.h>
#include <ros/console.h>
#include <nuturtle_robot/StartRotation.h>
#include "rigid2d/SetPose.h"
#include <geometry_msgs/Twist.h>
#include "rigid2d/rigid2d.hpp"

//constexpr double callTime=0.1;

class RotateInPlace
{
  ros::ServiceServer startRotation;
  ros::ServiceClient setPose;
  ros::Publisher cmdVelPUblisher;
  bool isClockwise, isWaitTime;
  double totalElapsedTime, rotationVelocity, callbackTime, rotationPeriod, fracVel, maxRotVelRobot;
  double totalTime;
  int rotationCount, callbackCount, rotationsNeeded;
  enum RotationStates {ROTATE, WAIT, STOP}; 
  RotationStates state;
  bool startService;
  ros::Time currentTime, lastTime;
  //double callbackTime;
  public:
  RotateInPlace(int argc, char** argv)
  {

    ros::init(argc, argv, "rotate_in_place");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    startRotation = n.advertiseService("/start", &RotateInPlace::startCallback,
                                     this);
    cmdVelPUblisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    setPose = n.serviceClient<rigid2d::SetPose>("/set_pose");
    ros::param::get("~frac_vel", fracVel);
    ros::param::get("max_rot_vel_robot", maxRotVelRobot);
    ROS_INFO_STREAM("fractional_velocity:  "<<fracVel);
    isClockwise = true;
    //callbackTime = callTime;
    totalElapsedTime = 0.0;
    rotationCount = 0;
    rotationVelocity = fracVel * maxRotVelRobot;
    isWaitTime = false;
    state = WAIT;
    rotationPeriod = 2.0 * rigid2d::PI / rotationVelocity;
    rotationsNeeded = 20;
    startService = false;
    callbackTime = rotationPeriod / 200.0;
  }

 double returnCallbackTime()
 {
   return callbackTime;
 }

  bool startCallback(nuturtle_robot::StartRotation::Request& request,
                     nuturtle_robot::StartRotation::Response& response)
  {
    if(setPose.exists())
    {
      rigid2d::SetPose initialPose;
      initialPose.request.desiredPose.x = 0;
      initialPose.request.desiredPose.y = 0;
      initialPose.request.desiredPose.theta = 0;
      setPose.call(initialPose);
    }
    isClockwise = request.isClockwise;
    if(isClockwise)
    {
      rotationVelocity = -rotationVelocity;
    }
    startService = true;
    return true;

  }

  void cmdVelPublishCallback(const ros::TimerEvent& event)
  {

    //double totalTime =  callbackCount * callbackTime;
    // double totalTime += 
    


    ROS_INFO_STREAM("total time"<< totalTime<<"rotationPeriod"<<rotationPeriod);
    ROS_INFO_STREAM("call back count"<< callbackCount);
    ROS_INFO_STREAM("state:"<<state);
    ROS_INFO_STREAM(startService);
    currentTime = ros::Time::now();

    if(not startService)
    {
    lastTime = currentTime;

    geometry_msgs::Twist rotationTwist;
    rotationTwist.linear.x = 0;
    rotationTwist.linear.y = 0;
    rotationTwist.linear.z = 0;
    rotationTwist.angular.x = 0;
    rotationTwist.angular.y = 0;
    rotationTwist.angular.z = 0;

    cmdVelPUblisher.publish(rotationTwist);

    return;
    }

    ros::Duration time_duration = currentTime - lastTime;
    totalTime += time_duration.toSec();

    if(rotationCount == rotationsNeeded)
    {
      state = STOP;
    }

    else if(state == ROTATE && (totalTime >= rotationPeriod))
    {
      //callbackCount = 0;
      totalTime = 0;
      rotationCount += 1;
      state = WAIT;
    }

    else if(state == WAIT && (totalTime >= rotationPeriod / 20))
    {
      //callbackCount = 0;
      totalTime = 0;

      state = ROTATE;
    }

    ROS_INFO_STREAM("total time"<< totalTime<<"rotationPeriod"<<rotationPeriod);
    ROS_INFO_STREAM("call back count"<< callbackCount);
    ROS_INFO_STREAM(state);

    geometry_msgs::Twist rotationTwist;
    rotationTwist.linear.x = 0;
    rotationTwist.linear.y = 0;
    rotationTwist.linear.z = 0;
    rotationTwist.angular.x = 0;
    rotationTwist.angular.y = 0;


    switch(state)
    {
      case ROTATE: rotationTwist.angular.z = rotationVelocity;
                   break;
      case WAIT: rotationTwist.angular.z = 0;
                 break;
      case STOP: rotationTwist.angular.z = 0;
                 break;
    };

    callbackCount += 1;

    cmdVelPUblisher.publish(rotationTwist);
    lastTime = currentTime;
  }
};



int main(int argc, char** argv)
{
  RotateInPlace turtleRotate(argc, argv);
  ros::NodeHandle n;
  ros::Timer timer = n.createTimer(ros::Duration(turtleRotate.returnCallbackTime()), &RotateInPlace::cmdVelPublishCallback,
                                   &turtleRotate);
  ros::spin();
  return 0;
}
