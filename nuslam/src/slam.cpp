/// \file
/// \brief  This file implements all the slam functionality of the turtlebot
///
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
///  /filter_output (nuslam/FilterOutput) - Output state estimate of the robot pose and landmark
///                                      estimates are published in this topic.
/// SUBSCRIBES:
///  joint_states (sensor_msgs/JointState)  - A topic where the position, velocity, torque
///                                           and all other states of the robot joints is 
///                                           published
///  /landmarks (nuslam/TurtleMap) - A topic where detected landmarks with their data association
///                                  are published.
/// SERVICE:
/// set_pose (rigid2d/SetPose) - A service for setting the initial pose of the robot

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

constexpr size_t landmarkCount = 50;




typedef Matrix<double, 2 * landmarkCount+3, 1> stateVector;
typedef Matrix<double, 2 * landmarkCount+3, 2 * landmarkCount+3> stateCovariance;
typedef Matrix<double, 2, 2 * landmarkCount+3> Measurement;
typedef Matrix<double, 2, 1> MeasurementVector;



/// \brief Checks if two given float values are almost equal
/// \param d1 - first number
/// \param d2 - second number
/// \epsilon - maximum difference allowed
/// \returns A boolean indicating where the two numbers are equal or not
bool almost_equal(double d1, double d2, double epsilon=1.0e-6)
{
   return(abs(d1-d2) < epsilon? true: false);
}

/// \brief Calculates the clockwise Distance distance between two angles
/// \param x - angle 1
/// \param y - angle 2
/// \returns anticlockwise distance between two angles
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


/// \brief Calculates the anticlockwise Distance distance between two angles
/// \param x - angle 1
/// \param y - angle 2
/// \returns anticlockwise distance between two angles
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
   /// \constructor
   /// \param argc - A command line argument indicating the number of arguments
   /// \param argv  - A command line argument containing the arguments
  slam(int argc, char** argv)
  {


     ros::init(argc, argv, "odometer");
     ros::NodeHandle n;
     tf::TransformBroadcaster odom_broadcaster;
     currentTime = ros::Time::now();
     lastTime = ros::Time::now();
     ros::Rate r(50);
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
    state.setZero(2 * landmarkCount + 3);

    // Motion model noise covariance
    Q.setZero(2 *landmarkCount + 3, 2 * landmarkCount + 3);
    Q(0,0) = 0.00000304617;
    Q(1,1) = 1e-6;
    Q(2,2) = 1e-6;


    // Measurement noise covariance
    R.setZero(2, 2);
    R(0,0) = 1e-8;
    R(1,1) = 1e-8;
    detectedLandmarks = 0;


    int i=0, j=0;

    for(;i<sigma.rows(); i++)
    {
      for(; j<sigma.cols(); j++)
      {
          sigma(i, j) = 0.0;

      }

    }
  }

  /// \brief A service call back function for setting the initial pose of the robot
  /// \param request - This contains the input data to the service. This contains the pose
  ///                  of the robot that is desired
  /// \param repsonse - An empty response
  /// \returns A boolean inidicating if the service failed or passed

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

  /// \brief A landmark callback. This is the main slam update function. For every measured
  ///        received, a slam update is made
  /// \param landmarkMessage - A message containing new measurements

  void landmarkCallback(const nuslam::TurtleMap& landmarkMessage)
  {
    auto startTime = ros::Time::now();

    if(landmarkMessage.landmarkCount > detectedLandmarks)
    {
      int index;
      for(index=detectedLandmarks; index < landmarkMessage.landmarkCount; index++)
      {
          size_t landmarkPosition = index;
          state(2 + 2 * landmarkPosition +1) = landmarkMessage.centerX[index];
          state(2 + 2 * landmarkPosition +2) = landmarkMessage.centerY[index];

          sigma(2 + 2 * landmarkPosition+1, 2 + 2 * landmarkPosition+1) = 0.1;
          sigma(2 + 2 * landmarkPosition+2, 2 + 2 * landmarkPosition+2) = 0.1;


      }
      detectedLandmarks = landmarkMessage.landmarkCount;
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
     
    size_t measurementIndex =0;

    for(;measurementIndex < landmarkMessage.landmarkIndex.size(); measurementIndex++)
    {

      size_t  landmarkIndex = landmarkMessage.landmarkIndex[measurementIndex];

      double centerX = state(2 + 2 * landmarkIndex + 1);
      double centerY = state(2 + 2 * landmarkIndex + 2);

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
      zt(0) = landmarkMessage.range[measurementIndex];
      zt(1) = landmarkMessage.bearing[measurementIndex];

      auto K = sigma * Ht.transpose() * (Ht * sigma * Ht.transpose() + R).inverse();
      difference = zt - ztBar;
      difference(1) = atan2(sin(difference(1)), cos(difference(1)));
      state = state + K * difference;
      state(0) = atan2(sin(state(0)), cos(state(0)));
      sigma = (stateCovariance::Identity(2 * landmarkCount + 3, 2 * landmarkCount + 3) - K * Ht) * sigma;
    }
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
    auto endTime = ros::Time::now();
    ROS_INFO_STREAM("Time for a slam update"<<endTime - startTime);
  }

  /// \brief A function that publishes the filter output estimations
  /// \param state - Current filter state which includes Robot pose and landmark locations
  /// \param messageTime - Ros time stamp at which the message is publishes
  /// \param landmarkCount - Number of landmarks detected

  void publishFilterOutput(const stateVector& state, const ros::Time& messageTime, const int& landmarkCount)
  {
    nuslam::FilterOutput filterOuput;
    filterOuput.header.stamp = messageTime;
    filterOuput.robotPose.x = state(1);
    filterOuput.robotPose.y = state(2);
    filterOuput.robotPose.theta = state(0);
    int i;
    for(i=0; i< landmarkCount; i++)
    {
    filterOuput.landmarkX.push_back(state(2+2*i+1));
    filterOuput.landmarkY.push_back(state(2+ 2*i + 2));
    filterOuput.landmarkr.push_back(0.04);
    }

    filterOpPublisher.publish(filterOuput);

  }

  /// \brief This function publishes the map to odom frame transform
  /// \param Tmo - The transformation between map and odom frame
  /// \param messageTime - Ros time stamp at which the measurement was received.

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

  /// \brief Joint states callback function. This is the function where odometry is calculated
  /// \param jointMessage - A message containing the joint states of the robot
  void jointStatesCallback(const sensor_msgs::JointState jointMessage)
  {


    currentTime = jointMessage.header.stamp;
    newLeftPosition = jointMessage.position[0];
    newRightPosition = jointMessage.position[1];


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
    double leftVelocity = jointMessage.velocity[0];
    double rightVelocity = jointMessage.velocity[1];

    rigid2d::WheelVelocities velocities = {leftVelocity, rightVelocity};
    bodyTwist = diffcar.WheelVelocitiestoTwist(velocities);

    carPose = diffcar.returnPose();


    diffcar.UpdateOdometry(leftVelocity * totalTime, rightVelocity * totalTime);
    carPose = diffcar.returnPose();

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
    newLeftPosition = jointMessage.position[0];
    newRightPosition = jointMessage.position[1];
  
  
  }

};

/// \brief The main function
/// \param argc - Command line argument indicating the number of arguments
/// \param argv - Command line argument containing the arguments
int main(int argc, char** argv)
{

  slam slamNode(argc, argv);
  ros::spin();
  return 0;
}



