#ifndef _TURTLEDRIVE_PLUGIN_HH_
#define _TURTLEDRIVE_PLUGIN_HH_
/// \file
/// \brief This file implements a plugin that publishes sensor data message and moves the
//   the turtlebot robot in gazebo simulation
///
/// PARAMETERS:
/// left_wheel_joint  (string):   Name of the left wheel joint
/// right_wheel_joint (string): Name of the right wheel joint
/// sensor_frequency  (double): Rate which sensor data needs to be published
/// sensor_topic      (string): Topic to publish the sensor data to
/// encoder_ticks_per_rev (int): Number of encoder ticks per revolution
/// max_motor_vel     (double): Maximum motor velocity in meters per second
/// max_motor_power   (double): Maximum power of the motor
/// max_wheel_command (int): Maximum wheel command for rotating wheels
/// wheel_radius      (double): Radius of the wheel
/// wheel_cmd_topic   (string): Topic to publish the wheel commands
///
/// PUBLISHES:
/// sensor_topic - (nuturtlebot/SensorData) - Sensor data about wheel encoders are published
///                                           in this topic. All the other sensor data are left blank.
/// SUBSCRIBES:
/// wheel_cmd_topic - (nuturtlebot/WheelCommands) - Topic from the wheel commands to move the turtlebot
///                                                 in gazebo simulation are published



#include <iostream>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include "nuturtlebot/WheelCommands.h" 
#include "nuturtlebot/SensorData.h" 
#include "rigid2d/rigid2d.hpp"

using std::cout;
using std::string;

namespace gazebo
{
  class TurtleDrivePlugin : public ModelPlugin
  {
    physics::ModelPtr model;
    string leftJoint, rightJoint;
    double sensorFrequency, maxMotorPower, maxMotorRotVel, wheelCmdToVelRatio, wheelRadius;
    int encoderTicksPerRev;
    ros::Publisher sensorMessagePublisher;
    ros::Subscriber wheelCmdSubscriber;
    ros::Timer timer;
    string sensorTopic, wheelCmdTopic;
    nuturtlebot::SensorData wheelData;
    int maxWheelCmd;
    public:
    /// \brief - A constructor for initializing the turtledriveplugin

    TurtleDrivePlugin (): ModelPlugin ()
      {
        std::cerr<<"Initialised";
        printf("Initialised\n");
      }

    /// \brief - Callback function for wheel commands topic
    /// \param wheelCmdMessage - A Message containing the wheel commands

    void wheelCmdCallback(const nuturtlebot::WheelCommands& wheelCmdMessage)
    {
      const auto leftWheelCmd = wheelCmdMessage.left_velocity;
      const auto rightWheelCmd = wheelCmdMessage.right_velocity;
      double leftVelocity = leftWheelCmd * wheelCmdToVelRatio;
      double rightVelocity = rightWheelCmd * wheelCmdToVelRatio;


      model->GetJoint(leftJoint)->SetParam("fmax", 0, 100.0);
      model->GetJoint(rightJoint)->SetParam("fmax", 0, 100.0);

      model->GetJoint(leftJoint)->SetParam("vel", 0, leftVelocity);
      model->GetJoint(rightJoint)->SetParam("vel", 0, rightVelocity);


    }

    /// \brief - A timer callback function for publishing encoder data
    /// \param event - Timer callback event

    void publishEncoderData(const ros::TimerEvent& event)
    {

      auto leftJointPosition = model->GetJoint(leftJoint)->Position(0);
      auto rightJointPosition = model->GetJoint(rightJoint)->Position(0);
      int leftEncoder = (int) floor(leftJointPosition / (2 * rigid2d::PI) *  encoderTicksPerRev);
      int rightEncoder = (int) floor(rightJointPosition / (2 * rigid2d::PI) * encoderTicksPerRev);
      wheelData.left_encoder = leftEncoder;
      wheelData.right_encoder = rightEncoder;
      wheelData.stamp = ros::Time::now();
      sensorMessagePublisher.publish(wheelData);

    }

    /// \brief - Intialisation function for the plugin
    /// \param _model - The model for which the plugin is written
    /// \param _sdf -  The robot decription sdf file
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {

        model = _model;
        if (!ros::isInitialized())
        {
          ROS_FATAL("A ROS node for Gazebo has not been initialized."
                    "Unable to load plugin. Load the Gazebo system plugin"
                    "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
          return;
        }


        if(! _sdf-> HasElement("left_wheel_joint"))
         {
         ROS_ERROR("left joint not specified");

         }
         else
         {
         leftJoint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
          std::cerr<<"left wheel joint name is"<<leftJoint<<"\n";

         }


        if(! _sdf-> HasElement("right_wheel_joint"))
         {
           ROS_ERROR("right joint not specified");

         }

         else
         {
         rightJoint =_sdf->GetElement("right_wheel_joint")->Get<std::string>();
          std::cerr<<"right wheel joint name is"<<rightJoint<<"\n";
         }

         if(! _sdf-> HasElement("sensor_frequency"))
         {

           sensorFrequency = 200.0;
           std::cerr<<"sensor frequency not specified. Using a default frequency of 200";

         }
         else
         {

           sensorFrequency = std::stod(_sdf->GetElement("sensor_frequency")->Get<std::string>());
         }


        if(! _sdf-> HasElement("sensor_topic"))
         {
         ROS_ERROR("sensor topic not specified");

         }
         else
         {
         leftJoint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
          std::cerr<<"left wheel joint name is"<<leftJoint<<"\n";

         }


        std::cerr<<"sensor frequency"<<sensorFrequency<<"\n";


        if (not ros::param::get("encoder_ticks_per_rev", encoderTicksPerRev))
        {
           ROS_ERROR("Failed to get param encoder_ticks_per_rev");
        }


        if (not ros::param::get("max_rot_vel_robot" , maxMotorRotVel))
          {
           ROS_ERROR("Failed to get maximum motor rotational velocity");
          }

        if (not ros::param::get("max_motor_power", maxMotorPower))
          {
           ROS_ERROR("Failed to get maximum motor power");
          }

        if (not ros::param::get("max_wheel_command", maxWheelCmd))
          {
           ROS_ERROR("Failed to get maximum wheel command");
          }


        if (not ros::param::get("wheel_radius", wheelRadius))
          {
           ROS_ERROR("Failed to get wheel radius");
          }


        if(! _sdf-> HasElement("wheel_cmd_topic"))
         {
         ROS_ERROR("wheel command not specified");

         }
         else
         {
         wheelCmdTopic = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
          std::cerr<<"wheel command topic is"<<wheelCmdTopic<<"\n";
         }

         if(! _sdf-> HasElement("sensor_topic"))
         {
         ROS_ERROR("sensor topic not specified");

         }
         else
         {
         sensorTopic = _sdf->GetElement("sensor_topic")->Get<std::string>();
         std::cerr<<"sensor topic is"<<sensorTopic<<"\n";
         }

         wheelCmdToVelRatio = maxMotorRotVel / (double) maxWheelCmd;


        std::cerr<<"load function";
        ros::NodeHandle n;

       sensorMessagePublisher = n.advertise<nuturtlebot::SensorData>(sensorTopic, 1000, true);
       wheelCmdSubscriber = n.subscribe(wheelCmdTopic, 10000, &TurtleDrivePlugin::wheelCmdCallback, this);

      timer = n.createTimer(ros::Duration(1.0 / (double) sensorFrequency),
                                          &TurtleDrivePlugin::publishEncoderData, this);
      }
  };
  GZ_REGISTER_MODEL_PLUGIN(TurtleDrivePlugin);
}
#endif
