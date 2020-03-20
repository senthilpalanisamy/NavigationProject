#include <vector>
#include <iostream>
#include <functional>
#include <numeric>
#include <ros/ros.h>
#include <ros/console.h>

#include <Eigen/Dense>

#include "nuslam/circle_detection.hpp"
#include "sensor_msgs/LaserScan.h"
#include "rigid2d/rigid2d.hpp"
#include "nuslam/circle_detection.hpp"
#include "nuslam/TurtleMap.h"
#include "nuslam/FilterOutput.h"
#include "geometry_msgs/Pose2D.h"


using std::cout;
using std::accumulate;
using rigid2d::Vector2D;
using rigid2d::distance;
using rigid2d::PI;
using Eigen::Vector3d;
using circleDetection::fitCircle;
using rigid2d::Transform2D;
using rigid2d::Vector2D;

enum LandmarkStates{ADD, REMOVE, UNKNOWN};



struct CircleParameters
{
  double centerX, centerY, radius, range, bearing;
};


struct LandmarkGuess
{
  CircleParameters landmark;
  size_t hit=0;
  size_t miss=0;
  int maxTries=10;
  double minDetectionRate = 0.5;
  double calculateDetectionRate()
  {
    return (double) hit / (double) (hit + miss);
  }

  LandmarkStates isFinalised()
  {
    if(hit+miss < maxTries)
    {
      return UNKNOWN;
    }
    else
    {
      if(calculateDetectionRate() > minDetectionRate)
      {
        return ADD;
      }
      else
      {
        return REMOVE;
      }
    }
  }

};

double unwarpAngles(double angle)
{
  if(angle < 0)
  {
    return 2 * PI + angle;
  }
  else
  {
    return angle;
  }
}

class LandmarkGuessList
{
  public:
  vector<LandmarkGuess> landmarkList;
  double intraDistance=0.2;
  double newLandmarkDist = 0.27;
  size_t i=0;

  void updateLandmarks(vector<CircleParameters> detectedLandmarks)
  {

    vector<size_t> landmarkIndices;

    for(auto measurementLandmark: detectedLandmarks)
    {

      bool isLandmarkinList=false;
      double minDistance = std::numeric_limits<double>::infinity();
      for(i=0; i <landmarkList.size();i++)
      {
        auto potentialLandmark = landmarkList[i];
        double distance = sqrt(pow(potentialLandmark.landmark.centerX- measurementLandmark.centerX, 2) +
                              (pow(potentialLandmark.landmark.centerY - measurementLandmark.centerY, 2)));
        vector<LandmarkGuess> newLandmarksList;

        if(distance < intraDistance)
        {
        landmarkIndices.push_back(i);
        landmarkList[i].landmark.centerX = (landmarkList[i].hit * landmarkList[i].landmark.centerX + measurementLandmark.centerX) / (double) (landmarkList[i].hit+1);
        landmarkList[i].hit += 1;
        isLandmarkinList = true;
        }
        else if(distance < minDistance)
        {
          minDistance = distance;
        }

      }
      if(not isLandmarkinList && minDistance > newLandmarkDist)
      {
        LandmarkGuess newLandmark;
        newLandmark.landmark.centerX = measurementLandmark.centerX;
        newLandmark.landmark.centerY = measurementLandmark.centerY;
        newLandmark.landmark.radius = measurementLandmark.radius;
        landmarkList.push_back(newLandmark);
        landmarkList[landmarkList.size()-1].hit += 1;
        landmarkIndices.push_back(landmarkList.size()-1);
      }

    }

    for(i=0; i<landmarkList.size(); i++)
    {
    auto it = std::find(landmarkIndices.begin(), landmarkIndices.end(), i);
    if(it == landmarkIndices.end())
    {
    landmarkList[i].miss += 1;
    }

    }
  }

  vector<CircleParameters> returnFinalisedLandmarks()
  {
  size_t i;
  vector<CircleParameters> finalisedLandmarks;
  vector<size_t> indicesTodelete;
  for(i=0; i<landmarkList.size(); i++)
  {
    switch(landmarkList[i].isFinalised())
    {
     case ADD: finalisedLandmarks.push_back(landmarkList[i].landmark);
               indicesTodelete.push_back(i);
               break;
     case REMOVE:indicesTodelete.push_back(i);
                 break;
     case UNKNOWN:break;
    }
  }


  for (auto idx = indicesTodelete.rbegin(); idx != indicesTodelete.rend(); ++idx)
  {

     landmarkList.erase(landmarkList.begin() + *idx);

  }
  return finalisedLandmarks;
  }



};


double anticlockwiseDistance(double x, double y)
{
  if(y >= x)
  {
    return y-x;
   }
  else
  {
    return 2 * PI -x + y;
  }

}


double clockwiseDistance(double x, double y)
{
  if(y <= x)
  {
    return x-y;
   }
  else
  {
    return x + 2 * PI -y ;
  }

}


 class LandmarkDetection
 {
   ros::Subscriber laserscanSubscriber, filterOutputSubscriber;
   ros::Publisher landmarkPublisher;
   double maxRadius, minRadius, maxAssociationDistance;
   geometry_msgs::Pose2D robotPose;
   vector<double> allLandmarksX;
   vector<double> allLandmarksY;
   vector<CircleParameters> newMeasurements;
   vector<size_t> newMeasurementIndex;
   LandmarkGuessList temporaryLandmarkList;


   public:
   LandmarkDetection(int argc, char** argv)
   {
     ros::init(argc, argv, "landmark_detection");
     ros::NodeHandle n;
     ros::Rate loop_rate(100);
     laserscanSubscriber = n.subscribe("/scan", 1000, &LandmarkDetection::laserCallback,
                                     this);
     landmarkPublisher = n.advertise<nuslam::TurtleMap>("/landmarks", 1000);
     filterOutputSubscriber = n.subscribe("/filter_output", 1000, &LandmarkDetection::filterOutputCallback, this);
     maxRadius = 0.06;
     minRadius = 0.02;
     robotPose.x = 0;
     robotPose.y = 0;
     robotPose.theta = 0;
     maxAssociationDistance = 0.2;

   }

   void filterOutputCallback(const nuslam::FilterOutput filterOutput)
   {
     robotPose = filterOutput.robotPose;
     size_t i= 0;
     for(i=0; i < filterOutput.landmarkX.size(); i++)
     {
       allLandmarksX[i] = filterOutput.landmarkX[i];
       allLandmarksY[i] = filterOutput.landmarkY[i];
     }
   }



   void laserCallback(const sensor_msgs::LaserScan& laserMessage)
   {
     nuslam::TurtleMap mapMessage;
     ros::NodeHandle n;
     mapMessage.header.stamp = ros::Time::now();
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
     double distanceThreshold = 0.03;

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

     vector<int> indicesTodelete;


     for(i=0; i < clusteredPoints.size(); i++)

     {
       if(clusteredPoints[i].size() < 4)
       {
         indicesTodelete.push_back(i);

       }

     }


      for (auto idx = indicesTodelete.rbegin(); idx != indicesTodelete.rend(); ++idx)
      {

         clusteredPoints.erase(clusteredPoints.begin() + *idx);

      }

      // for(auto index: indicesTodelete)
      // {
      //   clusteredPoints.erase(clusteredPoints.begin() + index);
      // }

      cout<<"finished";
      cout<<"cluster size"<<clusteredPoints.size();


      vector<CircleParameters> allCircleParams;
      indicesTodelete.clear();
      size_t clusterIdx;





      for(auto cluster: clusteredPoints)
      {
        auto circleParams = fitCircle(cluster);
        CircleParameters circle;
        circle.centerX= (double) circleParams[0];
        circle.centerY= (double) circleParams[1];
        circle.radius= (double) circleParams[2];
        allCircleParams.push_back(circle);
        // mapMessage.centerX.push_back((double) circleParams[0]);
        // mapMessage.centerY.push_back((double) circleParams[1]);
        // mapMessage.radius.push_back((double) circleParams[2]);
      }



     // // Circle classification to remove false positives



      //for(auto cluster: clusteredPoints)
      for(clusterIdx=0; clusterIdx < clusteredPoints.size(); clusterIdx++)
      {
        auto cluster = clusteredPoints[clusterIdx];
        double mean = 0;
        double std = 0;
        vector<double> angles;
        vector<double> Pangles;
        vector<size_t> endpointIndex;

        for(i=0; i<cluster.size(); i++)
        {
          double angle = atan2(cluster[i].x - allCircleParams[clusterIdx].centerX,
                                cluster[i].y - allCircleParams[clusterIdx].centerY);
          angles.push_back(unwarpAngles(angle));
        }

        double P1Angle = angles[0];
        size_t P1Index = 0;
        double P2Angle = angles[0];
        size_t P2Index = 0;

        for(i=1; i<angles.size(); i++)
        {
          double clkDistance = clockwiseDistance(P1Angle, angles[i]);
          double anticlkDistance = anticlockwiseDistance(P2Angle, angles[i]);
          if(clockwiseDistance(P2Angle, angles[i]) < clockwiseDistance(P2Angle, P1Angle))
            continue;
          if(clkDistance < anticlkDistance)
           {
            P1Angle = angles[i];
            P1Index = i;
           }
          else
          {
            P2Angle = angles[i];
            P2Index = i;
          }

        }
        // auto P1Index = std::min_element(angles.begin(),angles.end()) - angles.begin();
        auto P1 = cluster[P1Index];
        // auto P2Index = std::max_element(angles.begin(),angles.end()) - angles.begin();
        auto P2 = cluster[P2Index];
        endpointIndex.push_back(P1Index);
        endpointIndex.push_back(P2Index);

        // for cosine law
        double c = distance(P1, P2);
        //cluster.erase(remove(cluster.begin(), cluster.end(), endpointIndex), cluster.end());
        std::sort(endpointIndex.begin(), endpointIndex.end());

         for (auto idx = endpointIndex.rbegin(); idx != endpointIndex.rend(); ++idx)
         {

            cluster.erase(cluster.begin() + *idx);

         }

        for(auto P: cluster)
        {
          double b = distance(P1, P);
          double a = distance(P2, P);
          double P1_P_P2Angle = acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b));
          Pangles.push_back(P1_P_P2Angle);
        }
        mean = accumulate( Pangles.begin(), Pangles.end(), 0.0 ) / Pangles.size();
        for(auto sampleAngle: Pangles)
        {
          std += pow(sampleAngle - mean, 2);
        }
        std = sqrt(std / Pangles.size());
        // mean is between 90 - 135 degrees
        if(mean<0.52 || mean>3.2 || std > 0.5)
        {
          indicesTodelete.push_back(clusterIdx);
        }


      }


  //allCircleParams.erase(remove(allCircleParams.begin(), allCircleParams.end(), indicesTodelete), allCircleParams.end());
  for (auto idx = indicesTodelete.rbegin(); idx != indicesTodelete.rend(); ++idx)
  {
     allCircleParams.erase(allCircleParams.begin() + *idx);
  }

  // filter circles based on radius
  indicesTodelete.clear();
  for(i=0; i < allCircleParams.size(); i++)
  {
    if(allCircleParams[i].radius > maxRadius)
    {
      indicesTodelete.push_back(i);

    }

    if(allCircleParams[i].radius < minRadius)
    {
      indicesTodelete.push_back(i);
    }
  }


  for (auto idx = indicesTodelete.rbegin(); idx != indicesTodelete.rend(); ++idx)
  {
     allCircleParams.erase(allCircleParams.begin() + *idx);
  }



  Vector2D robotPosition = {robotPose.x, robotPose.y};

  Transform2D Twr(robotPosition, robotPose.theta);

  for(auto& finalCircles:allCircleParams)
  {

    Vector2D center = {finalCircles.centerX, finalCircles.centerY};
    auto centerW = Twr(center);
    finalCircles.centerX = centerW.x;
    finalCircles.centerY = centerW.y;
    finalCircles.range = sqrt(pow(centerW.x - robotPose.x, 2) + pow(centerW.y - robotPose.y, 2));
    finalCircles.bearing = atan2(robotPose.y - centerW.y, robotPose.x - centerW.x);
  }

  indicesTodelete.clear();
  newMeasurements.clear();
  newMeasurementIndex.clear();

  size_t k=0;
  for(; k< allCircleParams.size(); k++)
  {
    auto finalCircles = allCircleParams[k];
    size_t idx=0;
    for(;idx < allLandmarksX.size(); idx++)
    {
      double distance = sqrt(pow(allLandmarksX[idx] - finalCircles.centerX, 2) + pow(allLandmarksY[idx] -finalCircles.centerY, 2));
      if(distance < maxAssociationDistance)
      {
        newMeasurements.push_back(finalCircles);
        newMeasurementIndex.push_back(idx);
        indicesTodelete.push_back(k);
        break;
      }

    }

  }


  for (auto idx = indicesTodelete.rbegin(); idx != indicesTodelete.rend(); ++idx)
  {
     allCircleParams.erase(allCircleParams.begin() + *idx);
  }





  temporaryLandmarkList.updateLandmarks(allCircleParams);
  auto fixedLandmarks = temporaryLandmarkList.returnFinalisedLandmarks();
  for(auto landmark:fixedLandmarks)
  {
    allLandmarksX.push_back(landmark.centerX);
    allLandmarksY.push_back(landmark.centerY);
  }







  //for(auto finalCircles:)
  //size_t i;
  //
  mapMessage.header.stamp = ros::Time::now();
  for(i=0; i < newMeasurements.size(); i++)
  {
    auto finalCircles = newMeasurements[i];
    size_t idx = newMeasurementIndex[i];
    //mapMessage.centerX.push_back(finalCircles.centerX);
    //mapMessage.centerY.push_back(finalCircles.centerY);
    mapMessage.radius.push_back(finalCircles.radius);
    mapMessage.range.push_back(finalCircles.range);
    mapMessage.bearing.push_back(finalCircles.bearing);
    mapMessage.landmarkIndex.push_back(idx);
  }

  for(i=0; i< allLandmarksX.size(); i++)
  {
    mapMessage.centerX.push_back(allLandmarksX[i]);
    mapMessage.centerY.push_back(allLandmarksY[i]);
  }
  mapMessage.landmarkCount = allLandmarksX.size();



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

