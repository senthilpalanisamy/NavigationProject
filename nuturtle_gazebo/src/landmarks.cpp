#include "nuturtle_gazebo/circle_detection.hpp"
#include "sensor_msgs/LaserScan.h"

#include <ros/ros.h>
#include <ros/console.h>


 class LandmarkDetection
 {
   ros::Subscriber laserscanSubscriber;
   public:
   LandmarkDetection(int argc, char** argv)
   {
     ros::init(argc, argv, "landmark_detection");
     ros::NodeHandle n;
     ros::Rate loop_rate(100);
     laserscanSubscriber = n.subscribe("/scan", 1000, &LandmarkDetection::laserCallback,
                                     this);
   }

   void laserCallback(const sensor_msgs::LaserScan& laserMessage)
   {
     cout<<"received";
   }


};


 /// \brief The main function
int main(int argc, char** argv)
{
  LandmarkDetection detectLandmarks(argc, argv);
  ros::spin();
  return 0;
}

