/// \file
/// \brief This file contains all functions definitions for the updating the odometry of 
/// a differential drive robot based on the wheel velocities received.
/// PARAMETERS:
///  ~odom_frame_id (string)    - Name of the odometry frame
///  ~body_frame_id (string)    - Name of the baselink of the robot
///  ~left_wheel_joint (sring)  - Name of the left wheel joint
///  ~right_wheel_joint (string)- Name of the right wheel joint
///  wheel_base       (double)  - wheel to wheel distance
///  wheel_radius     (double)  - wheel radius
/// PUBLISHES:
///   /nav_msgs/odometry (nav_msgs/odometry) - The odometry of the robot is published in 
///                                            this topic. It published every time a message
///                                            is received from joint_states
/// SUBSCRIBES:
///  joint_states (sensor_msgs/JointState)  - A topic where the position, velocity, torque
///                                           and all other states of the robot joints is 
///                                           published
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/console.h>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include "rigid2d/SetPose.h"
#include "rigid2d/rigid2d.hpp"
#include "nuslam/TurtleMap.h"
#include "nuslam/FilterOutput.h"

using std::string;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::Matrix2d;
using rigid2d::Vector2D;
using std::cout;
using rigid2d::Transform2D;

constexpr size_t landmarkCount = 20;




typedef Matrix<double, 2 * landmarkCount+3, 1> stateVector;
typedef Matrix<double, 2 * landmarkCount+3, 2 * landmarkCount+3> stateCovariance;
typedef Matrix<double, 2, 2 * landmarkCount+3> Measurement;
typedef Matrix<double, 2, 1> MeasurementVector;




bool almost_equal(double d1, double d2, double epsilon=1.0e-6)
{
   return(abs(d1-d2) < epsilon? true: false);
}


double clockwiseDistance(double x, double y)
{
  if(y >= x)
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
  if(y <= x)
  {
    return y-x;
   }
  else
  {
    return y - 2 * rigid2d::PI -x;
  }

}



double calculateWheelVelocities(double previousPoint, double presentPoint, double time)
{

    double clkDistance = clockwiseDistance(previousPoint, presentPoint);
    double anticlkDistance = anticlockwiseDistance(previousPoint, presentPoint);

    // ROS_INFO_STREAM("clockwise_distance"<<clkDistance);
    // ROS_INFO_STREAM("anticlockwise_distance"<<anticlkDistance);
    double wheelVelocity;
    if(abs(anticlkDistance) < clkDistance)
    {
     wheelVelocity = anticlkDistance / time;
    }
    else
    {
      wheelVelocity = clkDistance/ time;
    }
    return wheelVelocity;

}



class slam
 {

   string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
   ros::Subscriber jointStataSubscriber, landmarkSubscriber;
   ros::Publisher odometryPublisher, filterOpPublisher;
   ros::ServiceServer setTurtlePose;
   double wheelBase, wheelRadius, previousLeftPosition, previousRightPosition;
   double lastMeasurementPosLeft, lastMeasurementPosRight;
   double leftVelocity, rightVelocity;
   rigid2d::DiffDrive diffcar, previousPose;
   double leftWheelPosition, rightWheelPosition;
   double newLeftPosition, newRightPosition;
   ros::Time currentTime, lastTime;
   bool bIsFirstRun;
   string nameSpace;
   stateVector state;
   stateCovariance sigma, Gt, Q;
   Matrix2d R;
   Measurement Ht;
   MeasurementVector ztBar, zt, difference;
   int detectedLandmarks;


   public:
  slam(int argc, char** argv)
  {


     ros::init(argc, argv, "odometer");
     ros::NodeHandle n;
     tf::TransformBroadcaster odom_broadcaster;
     currentTime = ros::Time::now();
     lastTime = ros::Time::now();
     ros::Rate r(100);
     ros::param::get("/wheel_base", wheelBase);
     ros::param::get("/wheel_radius", wheelRadius);
     nameSpace = ros::this_node::getNamespace();
     jointStataSubscriber = n.subscribe("/joint_states", 1000, &slam::jointStatesCallback,
                                      this);
     odometryPublisher = n.advertise<nav_msgs::Odometry>("/nav_msgs/odometry", 1000);
     filterOpPublisher = n.advertise<nuslam::FilterOutput>("/filter_output", 1000);
     landmarkSubscriber = n.subscribe("/landmarks", 1000, &slam::landmarkCallback, this);

     if (not ros::param::get("~odom_frame_id", odom_frame_id))
     {
      ROS_ERROR("Failed to get odom frame id");
     }


     if (not ros::param::get("~body_frame_id", body_frame_id))
     {
      ROS_ERROR("Failed to get body frame id");
     }

     if (not ros::param::get("~left_wheel_joint", left_wheel_joint))
     {
      ROS_ERROR("Failed to get left_wheel_joint");
     }

     if (not ros::param::get("~right_wheel_joint", right_wheel_joint))
     {
      ROS_ERROR("Failed to get right_wheel_joint");
     }


     lastTime = ros::Time::now();
     leftWheelPosition = 0.0;
     rightWheelPosition = 0.0;
     previousLeftPosition = 0.0;
     previousRightPosition = 0.0;
     rigid2d::Transform2D identityTransform(0);
     diffcar = rigid2d::DiffDrive(identityTransform, wheelBase, wheelRadius); 
     setTurtlePose = n.advertiseService("set_pose", &slam::setTurtlePoseCallback,
                                       this);
    bIsFirstRun = true;
    previousLeftPosition = -1;
    previousRightPosition = -1;
    newLeftPosition = -1;
    newLeftPosition = -1;


    // initialise state and covariance
    //state  = state::Zero();
    state.setZero(2 * landmarkCount + 3);

    // Motion model noise covariance
    Q.setZero(2 *landmarkCount + 3, 2 * landmarkCount + 3);
    Q(0,0) = 0.00030461768 * 4;
    Q(1,1) = 1e-4;
    Q(2,2) = 1e-4;


    // Measurement noise covariance
    R.setZero(2, 2);
    R(0,0) = 1e-6;
    R(1,1) = 0.00030461768;
    detectedLandmarks = 0;

    // sigma = stateCovariance::Zero();
    size_t i=0, j=0;

    //for(;i<sigma.rows(); i++)
    //{
    //  for(; j<sigma.cols(); j++)
    //  {
    //    if(i<3 && j<3)
    //    {
    //      sigma(i, j) = 0.0;
    //    }
    //    else
    //    {
    //      //sigma(i, j) = std::numeric_limits<double>::infinity();
    //      sigma(i, j) = 1000;
    //    }

    //  }

    //}

    for(;i<sigma.rows(); i++)
    {
      for(; j<sigma.cols(); j++)
      {
          sigma(i, j) = 0.0;

      }

    }
  }

  bool setTurtlePoseCallback(rigid2d::SetPose::Request& request,
                                       rigid2d::SetPose::Response& response)
  {


     rigid2d::Vector2D newPosition = {request.desiredPose.x, request.desiredPose.y};
     rigid2d::Transform2D newPose(newPosition, request.desiredPose.theta);
     rigid2d::DiffDrive diffcarNewPose(newPose, wheelBase, wheelRadius);
     diffcar = diffcarNewPose;
     ROS_INFO_STREAM("Inside service\n");
     return true;
  }

  void landmarkCallback(const nuslam::TurtleMap& landmarkMessage)
  {

    if(landmarkMessage.landmarkIndex.size() > detectedLandmarks)
    {
      size_t index;
      for(index=0; index < landmarkMessage.landmarkIndex.size(); index++)
      {
        if(landmarkMessage.landmarkIndex[index] > detectedLandmarks-1)
        {
          size_t landmarkPosition = landmarkMessage.landmarkIndex[index];
          state(2 + 2 * landmarkPosition +1) = landmarkMessage.centerX[index];
          state(2 + 2 * landmarkPosition +2) = landmarkMessage.centerY[index];
          detectedLandmarks += 1;

          size_t i=0;
          for(i=0; i <= 2 + 2 * detectedLandmarks; i++)
          {
            sigma(2 + 2 * landmarkPosition+1, i) = 0.1;
            sigma(i, 2 + 2 * landmarkPosition+1) = 0.1;

            sigma(2 + 2 * landmarkPosition+2, i) = 0.1;
            sigma(i, 2 + 2 * landmarkPosition+2) = 0.1;
          }
        }

      }
    }
    if(not(previousLeftPosition == -1 or newLeftPosition == -1) &&
       (not(almost_equal(previousLeftPosition, newLeftPosition) && almost_equal(previousRightPosition, newRightPosition))))
    {
      double leftVelocity = previousLeftPosition - newLeftPosition;
      double rightVelocity = previousRightPosition - newRightPosition;

      rigid2d::WheelVelocities velocities = {leftVelocity, rightVelocity};
      auto odomTwist = diffcar.WheelVelocitiestoTwist(velocities);
      double deltaX = odomTwist.vx;
      double deltaTheta = odomTwist.wz;
      double presentAngle = state(0);

      Gt =  stateCovariance::Identity(2 * landmarkCount + 3, 2 * landmarkCount + 3);

      if(deltaTheta != 0)
        {

        cout<<state(1);
        state(1) = state(1) - deltaX / deltaTheta * sin(presentAngle) + deltaX / deltaTheta * sin(presentAngle + deltaTheta);
        cout<<state(1);
        state(2) = state(2) + deltaX / deltaTheta * cos(presentAngle) - deltaX / deltaTheta * cos(presentAngle + deltaTheta);
        state(0) = state(0) + deltaTheta;
        state(0) = atan2(sin(state(0)), cos(state(0)));
        Gt(1,0) += - deltaX / deltaTheta * cos(presentAngle) + deltaX / deltaTheta * cos(presentAngle + deltaTheta);
        Gt(2,0) += -deltaX / deltaTheta * sin(presentAngle) + deltaX / deltaTheta * sin(presentAngle + deltaTheta);
        }
      else
        {
        state(1) = state(1) + deltaX * cos(presentAngle);
        state(2) = state(2) + deltaX * sin(presentAngle);
        Gt(1,0) += -deltaX * sin(presentAngle);
        Gt(2,0) += deltaX * cos(presentAngle);
        }

      sigma = Gt * sigma * Gt.transpose() + Q;
     }
    size_t measurementIndex =0;
    //auto sigmaMinus = sigma;

    for(;measurementIndex < landmarkMessage.centerX.size(); measurementIndex++)
    {
      double centerX = landmarkMessage.centerX[measurementIndex];
      double centerY = landmarkMessage.centerY[measurementIndex];

      size_t  landmarkIndex = landmarkMessage.landmarkIndex[measurementIndex];
      ztBar(0) = sqrt(pow(centerX - state(1), 2) + pow(centerY - state(2), 2));
      double angle = atan2(centerY - state(2), centerX - state(1)) - state(0);
      ztBar(1) = atan2(sin(angle), cos(angle));

      double dx = centerX - state(1);
      double dy = centerY - state(2);
      double d = pow(dx, 2) + pow(dy, 2);

      Ht.setZero(2, 2 * landmarkCount + 3);
      Ht(1, 0) = -1;
      Ht(0, 1)= -dx / sqrt(d);
      Ht(1, 1) = dy / d;
      Ht(0, 2) = -dy / sqrt(d);
      Ht(1, 2) = -dx / d;
      Ht(0, 2 + 2 * landmarkIndex + 1) = dx / sqrt(d);
      Ht(1, 2 + 2 * landmarkIndex + 1) = -dy / d;
      Ht(0, 2 + 2 * landmarkIndex + 2) = dy / sqrt(d);
      Ht(1, 2 + 2 * landmarkIndex + 2) = dx / d;
      //auto Ht * sigma * Ht.transpose() + R
      zt(0) = landmarkMessage.range[measurementIndex];
      zt(1) = landmarkMessage.bearing[measurementIndex];

      auto K = sigma * Ht.transpose() * (Ht * sigma * Ht.transpose() + R).inverse();
      difference = zt - ztBar;
      difference(1) = atan2(sin(difference(1)), cos(difference(1)));
      //difference(0) = 0.0;
      //difference(1) = 0.0;
      state = state + K * difference;
      state(0) = atan2(sin(state(0)), cos(state(0)));
      sigma = (stateCovariance::Identity(2 * landmarkCount + 3, 2 * landmarkCount + 3) - K * Ht) * sigma;
    }

    Vector2D translation(state(1), state(2));

    Transform2D Tmr(translation, state(0));
    auto odomPose = diffcar.returnPose();
    Vector2D odomTrans(odomPose.x, odomPose.y);
    Transform2D Tor(odomTrans, odomPose.theta);
    auto Tmo = Tmr * Tor.inv();
    publishMapOdomTransform(Tmo, landmarkMessage.header.stamp);
    publishFilterOutput(state, landmarkMessage.header.stamp, landmarkMessage.landmarkCount);



    newLeftPosition = previousLeftPosition;
    newRightPosition = previousRightPosition;
  }

  void publishFilterOutput(const stateVector& state, const ros::Time& messageTime, const int& landmarkCount)
  {
    nuslam::FilterOutput filterOuput;
    filterOuput.header.stamp = messageTime;
    filterOuput.robotPose.x = state(1);
    filterOuput.robotPose.y = state(2);
    filterOuput.robotPose.theta = state(0);
    size_t i;
    for(i=0; i< landmarkCount; i++)
    {
    filterOuput.landmarkX.push_back(state(2+2*i+1));
    filterOuput.landmarkY.push_back(state(2+ 2*i + 2));
    filterOuput.landmarkr.push_back(0.04);
    }

    filterOpPublisher.publish(filterOuput);

  }

  void publishMapOdomTransform(const Transform2D& Tmo, const ros::Time& messageTime)
  {

    auto pose = Tmo.displacement();
    static tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = messageTime;
    transformStamped.header.frame_id = "/map";
    transformStamped.child_frame_id = "/odom";
    transformStamped.transform.translation.x = pose.x;
    transformStamped.transform.translation.y = pose.y;
    transformStamped.transform.translation.z = 0.0;


    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(pose.theta);

    transformStamped.transform.rotation.x = odomQuat.x;
    transformStamped.transform.rotation.y = odomQuat.y;
    transformStamped.transform.rotation.z = odomQuat.z;
    transformStamped.transform.rotation.w = odomQuat.w;
    br.sendTransform(transformStamped);
  }


  void jointStatesCallback(const sensor_msgs::JointState jointMessage)
  {


    currentTime = jointMessage.header.stamp;
    newLeftPosition = jointMessage.position[0];
    newRightPosition = jointMessage.position[1];
    //ROS_INFO_STREAM("left position"<<leftDistance);
    //ROS_INFO_STREAM("right position"<<rightDistance);


    auto carPose = diffcar.returnPose();
    rigid2d::Twist2D bodyTwist;

    if(bIsFirstRun)
    {
      lastTime = currentTime;
      bodyTwist = {0.0, 0.0, 0.0};
      bIsFirstRun = false;
      previousLeftPosition = jointMessage.position[0];
      previousRightPosition = jointMessage.position[1];
    }
    else
    {
    ros::Duration time_duration = currentTime - lastTime;
    double totalTime = time_duration.toSec();
    //ROS_INFO_STREAM("previousLeftPosition"<<previousLeftPosition);
    //ROS_INFO_STREAM("previousRightPosition"<<previousRightPosition);
    //ROS_INFO_STREAM("newLeftPosition"<<newLeftPosition);
    //ROS_INFO_STREAM("newRightPosition"<<newRightPosition);
    //ROS_INFO_STREAM("time_duration"<<time_duration);

    //double leftVelocity = calculateWheelVelocities(previousLeftPosition, newLeftPosition, totalTime);
    //double rightVelocity = calculateWheelVelocities(previousRightPosition, newRightPosition, totalTime);
    double leftVelocity = jointMessage.velocity[0];
    double rightVelocity = jointMessage.velocity[1];

    //ROS_INFO_STREAM("left velocity"<<leftVelocity * totalTime);
    //ROS_INFO_STREAM("right velocity"<<rightVelocity * totalTime);
    rigid2d::WheelVelocities velocities = {leftVelocity, rightVelocity};
    bodyTwist = diffcar.WheelVelocitiestoTwist(velocities);
    //ROS_INFO_STREAM("Body Twist z x y"<<bodyTwist.wz<<" "<<bodyTwist.vx<<" "<<bodyTwist.vy);
    //ROS_INFO_STREAM("time taken:"<<totalTime);

    carPose = diffcar.returnPose();

    //ROS_INFO_STREAM("before car pose x"<<carPose.x);
    //ROS_INFO_STREAM("before car pose y"<<carPose.y);
    //ROS_INFO_STREAM("before car pose theta"<<carPose.theta);

    diffcar.UpdateOdometry(leftVelocity * totalTime, rightVelocity * totalTime);
    carPose = diffcar.returnPose();

    //ROS_INFO_STREAM("after car pose x"<<carPose.x);
    //ROS_INFO_STREAM("after car pose y"<<carPose.y);
    //ROS_INFO_STREAM("after car pose theta"<<carPose.theta);

    }
  
    ROS_INFO_STREAM("\n Publishing message");
  
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = odom_frame_id;
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(carPose.theta);
  
    //set the position
    odom.pose.pose.position.x = carPose.x;
    odom.pose.pose.position.y = carPose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;
  
  
    //set the velocity
    odom.child_frame_id = body_frame_id;
    odom.twist.twist.linear.x = bodyTwist.vx;
    odom.twist.twist.linear.y = bodyTwist.vy;
    odom.twist.twist.angular.z = bodyTwist.wz;
  
    //publish the message
    odometryPublisher.publish(odom);
  
    ROS_INFO_STREAM("\n Publishing transform");
  
    ROS_INFO_STREAM("car pose x"<<carPose.x);
    ROS_INFO_STREAM("car pose y"<<carPose.y);
    ROS_INFO_STREAM("car pose theta"<<carPose.theta);
    ROS_INFO_STREAM("presentPosition left"<<jointMessage.position[0]);
    ROS_INFO_STREAM("presentPosition right"<<jointMessage.position[1]);
  
    //static tf2_ros::TransformBroadcaster br;
    static tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped; 
    transformStamped.header.stamp = currentTime;
    transformStamped.header.frame_id = odom_frame_id;
    transformStamped.child_frame_id = body_frame_id;
    transformStamped.transform.translation.x = carPose.x;
    transformStamped.transform.translation.y = carPose.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = odomQuat.x;
    transformStamped.transform.rotation.y = odomQuat.y;
    transformStamped.transform.rotation.z = odomQuat.z;
    transformStamped.transform.rotation.w = odomQuat.w;
    br.sendTransform(transformStamped);
  
  
    lastTime = jointMessage.header.stamp ;
  
    ROS_INFO_STREAM("\n Finished");
    ROS_INFO_STREAM("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    //previousLeftPosition = jointMessage.position[0];
    //previousRightPosition = jointMessage.position[1];
    newLeftPosition = jointMessage.position[0];
    newRightPosition = jointMessage.position[1];
  
  
  }

};

int main(int argc, char** argv)
{

  slam slamNode(argc, argv);
  ros::spin();
  return 0;
}



