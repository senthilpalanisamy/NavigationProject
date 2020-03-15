#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>


#include "rigid2d/rigid2d.hpp"
#include "nuslam/TurtleMap.h"

using rigid2d::Transform2D;
using rigid2d::distance;

class DrawMap
{
  ros::Subscriber landmarkSubscriber;
  ros::Publisher markerPub;
  tf2_ros::Buffer tfBuffer;
  visualization_msgs::Marker marker;

  public:
  DrawMap(int argc, char** argv)
   {
     ros::init(argc, argv, "draw_map");

      ros::NodeHandle n;
     ros::Rate loop_rate(100);
     landmarkSubscriber = n.subscribe("/landmarks", 1000, &DrawMap::landmarkCallback,
                                      this);
    markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100,true);
   }

   void landmarkCallback(const nuslam::TurtleMap& mapMessage)
   {

    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;
    Transform2D mapBaseTf;
    try
      {
      //transformStamped = tfBuffer.lookupTransform("base_link", "base_scan", mapMessage.header.stamp);
      transformStamped = tfBuffer.lookupTransform("base_link", "base_scan", ros::Time::now());
      }
    catch (tf2::TransformException &ex) 
      {
      ROS_WARN("Could NOT transform map to landmark: %s", ex.what());
      }



    marker.header.frame_id = "/base_link";
    marker.header.stamp = mapMessage.header.stamp;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 0.2;

     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 0.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
     marker.lifetime = ros::Duration(10.0);

     while (markerPub.getNumSubscribers() < 1)
     {
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }

    for(size_t i=0; i< mapMessage.centerX.size(); i++)
    {

     marker.ns = "basic_shapes";
     marker.id = i;

     marker.scale.x = mapMessage.radius[i];
     marker.scale.y = mapMessage.radius[i];
     marker.pose.position.x = mapMessage.centerX[i];
     marker.pose.position.y = mapMessage.centerY[i];
     marker.pose.position.z = 0;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     markerPub.publish(marker);
    }
  


   }

};





 /// \brief The main function
int main(int argc, char** argv)
{
  DrawMap mapGenerator(argc, argv);
  ros::spin();
  return 0;
}

