#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "nuslam/TurtleMap.h"

using std::cout;
using std::string;

class Analysis
{
  public:
    ros::Subscriber modelStateSub;
    ros::Publisher landmarkPublisher;
    Analysis(int argc, char** argv)
    {

      ros::init(argc, argv, "analysis");
      ros::NodeHandle n;

      modelStateSub = n.subscribe("gazebo/model_states", 1000, &Analysis::modelStateCallback, this);
      landmarkPublisher = n.advertise<nuslam::TurtleMap>("/real/landmarks", 1000);
    }

    void modelStateCallback(const gazebo_msgs::ModelStates& modelStateMsg)
    {
      cout<<"success";
      size_t i;
      string objectName="cylinder";
      nuslam::TurtleMap mapMessage;
      mapMessage.header.stamp = ros::Time::now();


      for(i=0; i< modelStateMsg.name.size(); i++)
      {
        if(modelStateMsg.name[i].find(objectName) != std::string::npos)
        {
        mapMessage.landmarkIndex.push_back(i);
        mapMessage.centerX.push_back(modelStateMsg.pose[i].position.x);
        mapMessage.centerY.push_back(modelStateMsg.pose[i].position.y);
        mapMessage.radius.push_back(0.04);
        }
      }
      landmarkPublisher.publish(mapMessage);

    }

};



int main(int argc, char** argv)
{
  Analysis analysis(argc, argv);
  ros::spin();
  return 0;
}

