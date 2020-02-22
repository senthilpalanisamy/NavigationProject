#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include "nuturtlebot/WheelCommands.h" 
#include "nuturtlebot/SensorData.h" 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <chrono>
#include <cmath>

/// \file
/// Test cases for ros tesing all nodes in turtle interface file

constexpr float PI = 3.141592653589793238;


class TurtleInterface {

   public:

     ros::Publisher cmdVelPublisher, sensorDataPublisher;
     ros::Subscriber wheelCmdSubscriber, jspSubscriber;
     double maxMotorVelocity, robotMaxTransVel, robotMaxRotVel;
     double wheelBase, wheelRadius, encoderTicksPerRevolution;
     std::string leftWheelJoint, rightWheelJoint;
     bool twistMessageReceived, jspMessageReceived;
     int maxWheelCommand;
     nuturtlebot::WheelCommands receivedCommand;
     sensor_msgs::JointState receivedJoint;
     ros::NodeHandle n;

        TurtleInterface()
        {

         //ros::Rate loop_rate(10);
         ros::param::get("max_rot_vel_motor", maxMotorVelocity);
         ros::param::get("max_trans_vel", robotMaxTransVel);
         ros::param::get("max_rot_vel_motor", robotMaxRotVel);
         ros::param::get("wheel_base", wheelBase);
         ros::param::get("wheel_radius", wheelRadius);
         ros::param::get("~left_wheel_joint", leftWheelJoint);
         ros::param::get("~right_wheel_joint", rightWheelJoint);
         ros::param::get("encoder_ticks_per_rev", encoderTicksPerRevolution);
         ros::param::get("max_wheel_command", maxWheelCommand);

         cmdVelPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, true);
         sensorDataPublisher = n.advertise<nuturtlebot::SensorData>("/sensor_data", 1000, true);
         wheelCmdSubscriber = n.subscribe("/wheel_cmd", 1000, &TurtleInterface::wheelCmdCallback, this);
         jspSubscriber = n.subscribe("/joint_states", 1000, &TurtleInterface::jspCallback, this);
         twistMessageReceived = false;
         jspMessageReceived = true;
         //loop_rate.sleep();
        }



   void publishTwist(const geometry_msgs::Twist zeroRotTwist)
   {
     twistMessageReceived = false;

   cmdVelPublisher.publish(zeroRotTwist);

   }

   void publishSensorData(const nuturtlebot::SensorData turtleBotData)
   {
   sensorDataPublisher.publish(turtleBotData);
   jspMessageReceived = false;
   }



   void wheelCmdCallback(const nuturtlebot::WheelCommands movementCommand)
   {
     receivedCommand = movementCommand;
     twistMessageReceived = true;

   }


  void jspCallback(const sensor_msgs::JointState jointState)
  {
    receivedJoint = jointState;
    jspMessageReceived = true;

  }

};



   TEST(TurtleBotMovement, PureTransaltionVelocityCommand) 
   {
     TurtleInterface turtleInterface;
     double wheelRadius = turtleInterface.wheelRadius;
     double maxWheelCommand = turtleInterface.maxWheelCommand;
     double maxMotorVelocity = turtleInterface.maxMotorVelocity;

     ROS_INFO("Entered test case");
     geometry_msgs::Twist zeroRotTwist;

     zeroRotTwist.linear.x = 0.1;
     zeroRotTwist.linear.y = 0;
     zeroRotTwist.linear.z = 0;
     zeroRotTwist.angular.x = 0;
     zeroRotTwist.angular.y = 0;
     zeroRotTwist.angular.z = 0;

     turtleInterface.publishTwist(zeroRotTwist);

      while(not turtleInterface.twistMessageReceived)
       {

         ros::spinOnce();
      }

     turtleInterface.twistMessageReceived = false;

     ROS_INFO("Compare results");
     auto receivedCommand = turtleInterface.receivedCommand;
     int expected_output = round(0.1 / wheelRadius * maxWheelCommand / maxMotorVelocity);
     ASSERT_EQ(receivedCommand.left_velocity, expected_output)<<"Velocities commands don't match the"
                                                                "expected value for a pure translation";
     ASSERT_EQ(receivedCommand.right_velocity, expected_output)<<"Velocities commands don't match the"
                                                                "expected value for a pure translation";
   }


    TEST(TurtleBotMovement, PureRotationVelocityCommand)
    {

    TurtleInterface turtleInterface;
    double wheelRadius = turtleInterface.wheelRadius;
    double maxWheelCommand = turtleInterface.maxWheelCommand;
    double maxMotorVelocity = turtleInterface.maxMotorVelocity;
    double wheelBase = turtleInterface.wheelBase;




      ROS_INFO("Entered test case");
      geometry_msgs::Twist pureRotTwist;

      pureRotTwist.linear.x = 0;
      pureRotTwist.linear.y = 0;
      pureRotTwist.linear.z = 0;
      pureRotTwist.angular.x = 0;
      pureRotTwist.angular.y = 0;
      pureRotTwist.angular.z = PI/2;

      turtleInterface.publishTwist(pureRotTwist);

      while(not turtleInterface.twistMessageReceived)
      {

        ros::spinOnce();
      }
      turtleInterface.twistMessageReceived = false;
      ROS_INFO("Compare results");
      double expectedWheelMovement = wheelBase / 2.0 * PI/2;
      auto receivedCommand = turtleInterface.receivedCommand;

      int expected_output = round(expectedWheelMovement / wheelRadius * maxWheelCommand / maxMotorVelocity);
      ASSERT_EQ(receivedCommand.right_velocity, expected_output)<<"Velocities commands don't match the"
                                                                 "expected value for a pure rotation";
      ASSERT_EQ(receivedCommand.left_velocity, -expected_output)<<"Velocities commands don't match the"
                                                                 "expected value for a pure rotation";
    }



   TEST(TurtleBotMovement, RotataionPlusTranslationVelocityCommand) 
   {

    TurtleInterface turtleInterface;
    double wheelRadius = turtleInterface.wheelRadius;
    double maxWheelCommand = turtleInterface.maxWheelCommand;
    double maxMotorVelocity = turtleInterface.maxMotorVelocity;
    double wheelBase = turtleInterface.wheelBase;



     ROS_INFO("Entered test case");
     geometry_msgs::Twist Twist;

     Twist.linear.x = 0.01;
     Twist.linear.y = 0;
     Twist.linear.z = 0;
     Twist.angular.x = 0;
     Twist.angular.y = 0;
     Twist.angular.z = PI/4;
     turtleInterface.publishTwist(Twist);

     while(not turtleInterface.twistMessageReceived)
      {

        ros::spinOnce();
      }

      auto receivedCommand = turtleInterface.receivedCommand;

     turtleInterface.twistMessageReceived = false;

     ROS_INFO("Compare results");
     // ASSERT_TRUE(output != NULL)<<"Message not received for testing robot velocity commands";
     float smallCircleRadius = Twist.linear.x / PI * 4.0 - wheelBase / 2.0;
     float bigCircleRadius = Twist.linear.x / PI * 4.0 + wheelBase /2.0; 
     float expectedLeftWheelVelocity = PI / 4.0 * smallCircleRadius / wheelRadius;
     float expectedRightWheelVelocity = PI / 4.0 * bigCircleRadius / wheelRadius;	
     int expectedLeftWheelCommand = round(expectedLeftWheelVelocity * maxWheelCommand / maxMotorVelocity);
     int expectedRightWheelCommand = round(expectedRightWheelVelocity * maxWheelCommand / maxMotorVelocity);
     ASSERT_EQ(receivedCommand.right_velocity, expectedRightWheelCommand)<<"Velocities commands don't match the"
                                                                "expected value for rotation plus translation";
     ASSERT_EQ(receivedCommand.left_velocity, expectedLeftWheelCommand)<<"Velocities commands don't match the"
                                                                "expected value for rotation plus translation";
   }
 
 
  TEST(JointState, EncoderToJointStates)
 {

    TurtleInterface turtleInterface;
    double encoderTicksPerRevolution = turtleInterface.encoderTicksPerRevolution;


   nuturtlebot::SensorData turtleBotData1;
   turtleBotData1.left_encoder = 0;
   turtleBotData1.right_encoder = 100;
   auto firstTime = ros::Time::now();
   turtleBotData1.stamp = firstTime;
   turtleInterface.publishSensorData(turtleBotData1);
   while(not turtleInterface.jspMessageReceived)
    {
      ros::spinOnce();
    }
 
   auto firstJointStates = turtleInterface.receivedJoint;
   turtleInterface.jspMessageReceived = false;
 
 
   nuturtlebot::SensorData turtleBotData2;
   turtleBotData2.left_encoder = 100;
   turtleBotData2.right_encoder = 0;
   auto secondTime = ros::Time::now();
   turtleBotData2.stamp = secondTime;
 
   turtleInterface.publishSensorData(turtleBotData2);
   while(not turtleInterface.jspMessageReceived)
    {
      ros::spinOnce();
    }
 
   auto secondJointStates = turtleInterface.receivedJoint;
   turtleInterface.jspMessageReceived = false;
 
 
   ros::Duration totalDuration = secondTime - firstTime;
   double totalTime = totalDuration.toSec();
 
   ASSERT_NEAR(firstJointStates.position[0],
               double(turtleBotData1.left_encoder) / encoderTicksPerRevolution * PI * 2,
               0.01)<<"Error in calculating joint states from sensor message";
   ASSERT_NEAR(firstJointStates.position[1],
               double(turtleBotData1.right_encoder) / encoderTicksPerRevolution * PI * 2,
               0.01)<<"Error in calculating joint states from sensor reading";
 
   ASSERT_NEAR(secondJointStates.position[0],
               double(turtleBotData2.left_encoder) / encoderTicksPerRevolution * PI * 2,
               0.01)<<"Error in calculating joint states from sensor message";
   ASSERT_NEAR(secondJointStates.position[1],
               double(turtleBotData2.right_encoder) / encoderTicksPerRevolution * PI * 2,
               0.01)<<"Error in calculating joint states from sensor reading";
 
 
   ASSERT_NEAR(secondJointStates.velocity[0],
               double(100) / encoderTicksPerRevolution * PI * 2 / totalTime,
               0.01)<<"Error in calculating joint states from sensor message";
   ASSERT_NEAR(secondJointStates.velocity[1],
               double(-100) / encoderTicksPerRevolution * PI * 2 / totalTime,
               0.01)<<"Error in calculating joint states from sensor reading";
 
 }







int main(int argc, char** argv)
{

   ros::init(argc, argv, "turtle_interface_test");
   ::testing::InitGoogleTest(&argc, argv);
   ROS_INFO("stated node");
   return RUN_ALL_TESTS();

  // testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "TurtleInterfaceTest");

  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  // int ret = RUN_ALL_TESTS();
  // spinner.stop();
  // ros::shutdown();
  // return ret;
}




