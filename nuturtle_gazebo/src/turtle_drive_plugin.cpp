#ifndef _TURTLEDRIVE_PLUGIN_HH_
#define _TURTLEDRIVE_PLUGIN_HH_

#include<iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include "nuturtlebot/WheelCommands.h" 
#include "nuturtlebot/SensorData.h" 

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
    string sensorTopic, wheelCmdTopic;
    int maxWheelCmd;
    public: 
    
    TurtleDrivePlugin (): ModelPlugin ()
      {
        std::cerr<<"Initialised";
        printf("Initialised\n");
      }

    void wheelCmdCallback(const nuturtlebot::WheelCommands& wheelCmdMessage)
    {
      const auto leftWheelCmd = wheelCmdMessage.left_velocity;
      const auto rightWheelCmd = wheelCmdMessage.right_velocity;
      double leftVelocity = leftWheelCmd * wheelCmdToVelRatio;
      double rightVelocity = rightWheelCmd * wheelCmdToVelRatio;

      std::cerr<<"executing call back\n leftvelocity:"<<leftVelocity<<"\nrightvelocity"<<rightVelocity;

      model->GetJoint(leftJoint)->SetParam("vel", 0, leftVelocity);
      model->GetJoint(rightJoint)->SetParam("vel", 0, rightVelocity);


    }

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

         wheelCmdToVelRatio = (double) maxWheelCmd / maxMotorRotVel * wheelRadius;


        std::cerr<<"load function";
        ros::NodeHandle n;

       sensorMessagePublisher = n.advertise<nuturtlebot::SensorData>(sensorTopic, 1000, true);
       wheelCmdSubscriber = n.subscribe(wheelCmdTopic, 10000, &TurtleDrivePlugin::wheelCmdCallback, this);
      }
  };
  GZ_REGISTER_MODEL_PLUGIN(TurtleDrivePlugin);
}
#endif
