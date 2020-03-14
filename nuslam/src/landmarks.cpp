#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Dense>

#include "nuslam/circle_detection.hpp"
#include "sensor_msgs/LaserScan.h"
#include "rigid2d/rigid2d.hpp"
#include "nuslam/circle_detection.hpp"
#include "nuslam/TurtleMap.h"

using std::cout;
using rigid2d::Vector2D;
using rigid2d::distance;
using Eigen::Vector3d;
using circleDetection::fitCircle;


 class LandmarkDetection
 {
   ros::Subscriber laserscanSubscriber;
   ros::Publisher landmarkPublisher;
   public:
   LandmarkDetection(int argc, char** argv)
   {
     ros::init(argc, argv, "landmark_detection");
     ros::NodeHandle n;
     ros::Rate loop_rate(100);
     laserscanSubscriber = n.subscribe("/scan", 1000, &LandmarkDetection::laserCallback,
                                     this);
     landmarkPublisher = n.advertise<nuslam::TurtleMap>("/landmarks", 1000);

   }



  // void odometryCallback(const nav_msgs::Odometry odometryMessage)
  // {

  //   current_x = odometryMessage.pose.pose.position.x;
  //   current_y = odometryMessage.pose.pose.position.y;
  //   tf::Quaternion q(
  //       odometryMessage.pose.pose.orientation.x,
  //       odometryMessage.pose.pose.orientation.y,
  //       odometryMessage.pose.pose.orientation.z,
  //       odometryMessage.pose.pose.orientation.w);
  //     tf::Matrix3x3 m(q);
  //     double roll, pitch, yaw;
  //     m.getRPY(roll, pitch, yaw); 
  //   current_angle = yaw;

  // }


   void laserCallback(const sensor_msgs::LaserScan& laserMessage)
   {
     nuslam::TurtleMap mapMessage;
     cout<<"received";
     auto ranges = laserMessage.ranges;
     auto angleIncrement = laserMessage.angle_increment;
     vector<Vector2D> detectedPoints;
     size_t i=0, j=0;
     Vector2D point;
     double theta = 0;
     //  Converting from r, theta to x, y
     for(; i< ranges.size(); i++)
     {
       point.x = ranges[i] * cos(theta);
       point.y = ranges[i] * sin(theta);
       detectedPoints.push_back(point);
       theta += angleIncrement;
     }

     // Converting from robot frame to world frame
     //
     vector<vector<Vector2D>> clusteredPoints;
     size_t clusterIndex;
     double distanceThreshold = 0.1;

     for(auto pointr: detectedPoints)
     {

       double minDistance = std::numeric_limits<double>::infinity();

       for(i=0; i< clusteredPoints.size(); i++)
       {
         auto cluster = clusteredPoints[i];
         for(j=0;j<cluster.size(); j++)
         {
           double eucledianDistance = distance(pointr, cluster[j]);
           if(eucledianDistance < minDistance)
           {
             minDistance = eucledianDistance;
             clusterIndex = i;
           }
         }
       }

      if(minDistance > distanceThreshold)
      {
       vector<Vector2D> newCluster;
       newCluster.push_back(pointr);
       clusteredPoints.push_back(newCluster);
      }
      else
      {
      clusteredPoints[clusterIndex].push_back(pointr);

      }


     }

     vector<size_t> indicesTodelete;


     for(i=0; i < clusteredPoints.size(); i++)

     {
       if(clusteredPoints[i].size() < 4)
       {
         indicesTodelete.push_back(i);

       }

     }

     for(auto index: indicesTodelete)
     {
       clusteredPoints.erase(clusteredPoints.begin() + index);
     }

     cout<<"finished";
     cout<<"cluster size"<<clusteredPoints.size();


     vector<Vector3d> clusterCircleParams;
     vector<double> centerX;
     vector<double> centerY;
     vector<double> radius;
     for(auto cluster: clusteredPoints)
     {
       auto circleParams = fitCircle(cluster);
       mapMessage.centerX.push_back((double) circleParams[0]);
       mapMessage.centerY.push_back((double) circleParams[1]);
       mapMessage.radius.push_back((double) circleParams[2]);
     }

  // mapMessage.centerX = centerX;
  // mapMessage.centerY = centerY;
  // mapMessage.radius = radius;
  landmarkPublisher.publish(mapMessage);



   }


};


 /// \brief The main function
int main(int argc, char** argv)
{
  LandmarkDetection detectLandmarks(argc, argv);
  ros::spin();
  return 0;
}

