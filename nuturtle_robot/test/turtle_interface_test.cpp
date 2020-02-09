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
//#include <turtle_interface.cpp>

// TODO: add other includes as needed
//

constexpr float PI = 3.141592653589793238;


class TurtleInterfaceFixture : public ::testing::Test {

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

        TurtleInterfaceFixture()
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
         wheelCmdSubscriber = n.subscribe("/wheel_commands", 1000, &TurtleInterfaceFixture::wheelCmdCallback, this);
         jspSubscriber = n.subscribe("/joint_states", 1000, &TurtleInterfaceFixture::jspCallback, this);
         twistMessageReceived = false;
         jspMessageReceived = true;
         //loop_rate.sleep();
        }



   void publishTwist(const geometry_msgs::Twist zeroRotTwist)
   {

   cmdVelPublisher.publish(zeroRotTwist);
   twistMessageReceived = false;

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



  TEST_F(TurtleInterfaceFixture, PureTransaltionVelocityCommand) 
  {



    ROS_INFO("Entered test case");
    geometry_msgs::Twist zeroRotTwist;

    zeroRotTwist.linear.x = 0.1;
    zeroRotTwist.linear.y = 0;
    zeroRotTwist.linear.z = 0;
    zeroRotTwist.angular.x = 0;
    zeroRotTwist.angular.y = 0;
    zeroRotTwist.angular.z = 0;
    publishTwist(zeroRotTwist);
    //cmdVelPublisher.publish(zeroRotTwist);

    // auto output = ros::topic::waitForMessage<nuturtlebot::WheelCommands>(wheelCmdSubscriber.getTopic(),
    //                                                                      ros::Duration(10));

     while(not twistMessageReceived)
      {

        ros::spinOnce();
      }

    twistMessageReceived = false;

    ROS_INFO("Compare results");
    //ASSERT_TRUE(output != NULL)<<"Message not received for testing robot velocity commands";
    int expected_output = round(0.1 / wheelRadius * maxWheelCommand / maxMotorVelocity);
    ASSERT_EQ(receivedCommand.left_velocity, expected_output)<<"Velocities commands don't match the"
                                                               "expected value for a pure translation";
    ASSERT_EQ(receivedCommand.right_velocity, expected_output)<<"Velocities commands don't match the"
                                                               "expected value for a pure translation";
  }


   TEST_F(TurtleInterfaceFixture, PureRotationVelocityCommand) 
   {
  
   
     ROS_INFO("Entered test case");
     geometry_msgs::Twist pureRotTwist;
  
     pureRotTwist.linear.x = 0;
     pureRotTwist.linear.y = 0;
     pureRotTwist.linear.z = 0;
     pureRotTwist.angular.x = 0;
     pureRotTwist.angular.y = 0;
     pureRotTwist.angular.z = PI;
     publishTwist(pureRotTwist);
  
     // auto output = ros::topic::waitForMessage<nuturtlebot::WheelCommands>(wheelCmdSubscriber.getTopic(),
     //                                                                      ros::Duration(10));
     while(not twistMessageReceived)
     {

       ros::spinOnce();
     }
     twistMessageReceived = false;
     ROS_INFO("Compare results");
     // ASSERT_TRUE(output != NULL)<<"Message not received for testing robot velocity commands";
     float expectedWheelMovement = wheelBase / 2.0 * PI;
     int expected_output = round(expectedWheelMovement / wheelRadius * maxWheelCommand / maxMotorVelocity);
     ASSERT_EQ(receivedCommand.right_velocity, expected_output)<<"Velocities commands don't match the"
                                                                "expected value for a pure rotation";
     ASSERT_EQ(receivedCommand.left_velocity, -expected_output)<<"Velocities commands don't match the"
                                                                "expected value for a pure rotation";
     //ASSERT_EQ(0, expected_output)<<"Velocities commands don't match the"
     //                                                           "expected value for a pure rotation";
   }



  TEST_F(TurtleInterfaceFixture, RotataionPlusTranslationVelocityCommand) 
  {
 
  
    ROS_INFO("Entered test case");
    geometry_msgs::Twist Twist;
 
    Twist.linear.x = 0.01;
    Twist.linear.y = 0;
    Twist.linear.z = 0;
    Twist.angular.x = 0;
    Twist.angular.y = 0;
    Twist.angular.z = PI;
    publishTwist(Twist);
 
    // auto output = ros::topic::waitForMessage<nuturtlebot::WheelCommands>(wheelCmdSubscriber.getTopic(),
    //                                                                      ros::Duration(10));
    while(not twistMessageReceived)
     {

       ros::spinOnce();
     }
    twistMessageReceived = false;
 
    ROS_INFO("Compare results");
    // ASSERT_TRUE(output != NULL)<<"Message not received for testing robot velocity commands";
    float smallCircleRadius = Twist.linear.x / PI - wheelBase / 2.0;
    float bigCircleRadius = Twist.linear.x / PI + wheelBase /2.0; 
    float expectedLeftWheelVelocity = PI * smallCircleRadius / wheelRadius;
    float expectedRightWheelVelocity = PI * bigCircleRadius / wheelRadius;	
    int expectedLeftWheelCommand = round(expectedLeftWheelVelocity * maxWheelCommand / maxMotorVelocity);
    int expectedRightWheelCommand = round(expectedRightWheelVelocity * maxWheelCommand / maxMotorVelocity);
    ASSERT_EQ(receivedCommand.right_velocity, expectedRightWheelCommand)<<"Velocities commands don't match the"
                                                               "expected value for rotation plus translation";
    ASSERT_EQ(receivedCommand.left_velocity, expectedLeftWheelCommand)<<"Velocities commands don't match the"
                                                               "expected value for rotation plus translation";
  }


 TEST_F(TurtleInterfaceFixture, EncoderToJointStates)
{
  nuturtlebot::SensorData turtleBotData1;
  turtleBotData1.left_encoder = 0;
  turtleBotData1.right_encoder = 100;
  auto firstTime = ros::Time::now();
  turtleBotData1.stamp = firstTime;
  publishSensorData(turtleBotData1);
  while(not jspMessageReceived)
   {
     ros::spinOnce();
   }

  auto firstJointStates = receivedJoint;
  jspMessageReceived = false;


  nuturtlebot::SensorData turtleBotData2;
  turtleBotData2.left_encoder = 100;
  turtleBotData2.right_encoder = 0;
  auto secondTime = ros::Time::now();
  turtleBotData2.stamp = secondTime;

  publishSensorData(turtleBotData2);
  while(not jspMessageReceived)
   {
     ros::spinOnce();
   }

  auto secondJointStates = receivedJoint;
  jspMessageReceived = false;


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




