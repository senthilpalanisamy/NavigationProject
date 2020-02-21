/// \file
/// \brief This file implements a node that publishes cmd_vel for making the turtlebot
/// translate
///
/// PARAMETERS:
/// ~frac_vel(double) : Fraction of the maximum velocity to use
///  max_rot_vel_robot(double) : Maximum rotation velocity of the robot
///
/// PUBLISHES:
///  cmd_vel (geometry_msgs/Twist) : A topic for publishing the body twist velocities
///                                  for rotating in place.
/// SERVICES:
/// set_pose (rigid2d/SetPose) : The pose of the robot can be set to any required pose.
/// start   (nuturtle_robot/StartRotation) : A boolean indicting if the node should 
///                                          start publishing cmd velocities. This is 
///                                          service is used for initiating the whole
///                                          process
#include <ros/ros.h>
#include <ros/console.h>
#include <nuturtle_robot/StartTranslation.h>
#include "rigid2d/SetPose.h"
#include <geometry_msgs/Twist.h>
#include "rigid2d/rigid2d.hpp"

constexpr double distancetoTravel=2.0;

class Translation
{
  ros::ServiceServer startRotation;
  ros::ServiceClient setPose;
  ros::Publisher cmdVelPUblisher;
  bool isForward, isWaitTime;
  double totalElapsedTime, transVelocity, callbackTime, translationPeriod, fracVel, maxTransVelRot;
  double totalTime;
  int stepCount, callbackCount, NoOfSteps;
  enum RotationStates {TRANSLATE, WAIT, STOP};
  RotationStates state;
  bool startService;
  ros::Time currentTime, lastTime;

  /// \brief A parameterised constructor for initialising the node. This constructor initialises
  /// all necessary parameters for running the node.
  /// \param argc- Number of command line arguments argv-All command line arguments
  public:
  Translation(int argc, char** argv)
  {

    ros::init(argc, argv, "translate");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    startRotation = n.advertiseService("/start", &Translation::startCallback,
                                     this);
    cmdVelPUblisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    setPose = n.serviceClient<rigid2d::SetPose>("/set_pose");
    ros::param::get("~frac_vel", fracVel);
    ros::param::get("max_trans_vel", maxTransVelRot);
    ROS_INFO_STREAM("fractional_velocity:  "<<fracVel);
    isForward = true;
    totalElapsedTime = 0.0;
    stepCount = 0;
    transVelocity = fracVel * maxTransVelRot;
    isWaitTime = false;
    state = WAIT;
    NoOfSteps = 10;
    translationPeriod = distancetoTravel / double(NoOfSteps) / transVelocity;

    ROS_INFO_STREAM("translationPeriod:"<<translationPeriod);
    ROS_INFO_STREAM("No of Steps:  "<<NoOfSteps);
    ROS_INFO_STREAM("transVelocity"<<transVelocity);
    ROS_INFO_STREAM("distancetoTravel:  "<<distancetoTravel);
    startService = false;
    callbackTime = translationPeriod / 200.0;
  }

 ///  \brief A getter function for returning the calculated callback time
 double returnCallbackTime()
 {
   return callbackTime;
 }


  /// \brief Callback function for setpose service
  /// \param request - Pose of the robot desired
  /// \param response - A boolean indicating if the service was executed successfully.
  bool startCallback(nuturtle_robot::StartTranslation::Request& request,
                     nuturtle_robot::StartTranslation::Response& response)
  {
    if(setPose.exists())
    {
      rigid2d::SetPose initialPose;
      initialPose.request.desiredPose.x = 0;
      initialPose.request.desiredPose.y = 0;
      initialPose.request.desiredPose.theta = 0;
      setPose.call(initialPose);
    }
    isForward = request.isForward;
    if(not isForward)
    {
      transVelocity = -transVelocity;
    }
    startService = true;
    return true;

  }

  /// A timer callback, which publishes cmd velocities for making the robot rotate in
  /// place
  /// \param event - A ros timer event which contains timing debugging information

  void cmdVelPublishCallback(const ros::TimerEvent& event)
  {

    //double totalTime =  callbackCount * callbackTime;
    // double totalTime += 
    


    ROS_INFO_STREAM("total time"<< totalTime<<"translationPeriod"<<translationPeriod);
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

    if(stepCount == NoOfSteps)
    {
      state = STOP;
    }

    else if(state == TRANSLATE && (totalTime >= translationPeriod))
    {
      //callbackCount = 0;
      totalTime = 0;
      stepCount += 1;
      state = WAIT;
    }

    else if(state == WAIT && (totalTime >= translationPeriod / 20))
    {
      //callbackCount = 0;
      totalTime = 0;

      state = TRANSLATE;
    }

    ROS_INFO_STREAM("total time"<< totalTime<<"translationPeriod"<<translationPeriod);
    ROS_INFO_STREAM("call back count"<< callbackCount);
    ROS_INFO_STREAM(state);

    geometry_msgs::Twist translationTwist;
    translationTwist.linear.y = 0;
    translationTwist.linear.z = 0;
    translationTwist.angular.x = 0;
    translationTwist.angular.y = 0;
    translationTwist.angular.z = 0;


    switch(state)
    {
      case TRANSLATE: translationTwist.linear.x = transVelocity;
                   break;
      case WAIT: translationTwist.linear.x = 0;
                 break;
      case STOP: translationTwist.linear.x = 0;
                 break;
    };

    callbackCount += 1;

    cmdVelPUblisher.publish(translationTwist);
    lastTime = currentTime;
  }
};


/// \brief The main function
int main(int argc, char** argv)
{
  Translation turtleTranslate(argc, argv);
  ros::NodeHandle n;
  ros::Timer timer = n.createTimer(ros::Duration(turtleTranslate.returnCallbackTime()), &Translation::cmdVelPublishCallback,
                                   &turtleTranslate);
  ros::spin();
  return 0;
}
