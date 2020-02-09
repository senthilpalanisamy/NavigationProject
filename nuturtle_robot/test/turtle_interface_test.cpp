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


class TestNode : public ::testing::Test {

   public:

     ros::Publisher cmdVelPublisher, sensorDataPublisher;
     ros::Subscriber wheelCmdSubscriber, jspSubscriber;
     double maxMotorVelocity, robotMaxTransVel, robotMaxRotVel;
     double wheelBase, wheelRadius, encoderTicksPerRevolution;
     std::string leftWheelJoint, rightWheelJoint;
     bool message_received;
     int maxWheelCommand;
     nuturtlebot::WheelCommands receivedCommand;
     sensor_msgs::JointState receivedJoint;
     ros::NodeHandle n;

        TestNode()
        {

         ros::Rate loop_rate(10);
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
         wheelCmdSubscriber = n.subscribe("/wheel_commands", 1000, &TestNode::wheelCmdCallback, this);
         jspSubscriber = n.subscribe("/joint_states", 1000, &TestNode::jspCallback, this);
         message_received = false;
        }



   void publish(geometry_msgs::Twist zeroRotTwist)
   {

   cmdVelPublisher.publish(zeroRotTwist);

   }



   void wheelCmdCallback(const nuturtlebot::WheelCommands movementCommand)
   {
     receivedCommand = movementCommand;

   }


  void jspCallback(const sensor_msgs::JointState jointState)
  {
    receivedJoint = jointState;

  }

};



 TEST_F(TestNode, TwistZeroRotation) 
 {



   ROS_INFO("Entered test case");
   geometry_msgs::Twist zeroRotTwist;

   zeroRotTwist.linear.x = 0.1;
   zeroRotTwist.linear.y = 0;
   zeroRotTwist.linear.z = 0;
   zeroRotTwist.angular.x = 0;
   zeroRotTwist.angular.y = 0;
   zeroRotTwist.angular.z = 0;
   cmdVelPublisher.publish(zeroRotTwist);

   auto output = ros::topic::waitForMessage<nuturtlebot::WheelCommands>(wheelCmdSubscriber.getTopic(),
                                                                        ros::Duration(1));

   ROS_INFO("Compare results");
   ASSERT_TRUE(output != NULL)<<"Message not received for testing robot velocity commands";
   int expected_output = round(0.1 / wheelRadius * maxWheelCommand / maxMotorVelocity);
   ASSERT_NEAR(output->left_velocity, expected_output, 0.01)<<"Velocities commands don't match the"
                                                              "expected value for a pure translation";
   ASSERT_NEAR(output->right_velocity, expected_output, 0.01)<<"Velocities commands don't match the"
                                                              "expected value for a pure translation";
 }





int main(int argc, char** argv)
{

  ros::init(argc, argv, "turtle_interface_test");
  ::testing::InitGoogleTest(&argc, argv);
  ROS_INFO("stated node");
  return RUN_ALL_TESTS();
}




