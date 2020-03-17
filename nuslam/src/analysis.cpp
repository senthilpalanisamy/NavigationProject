#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>

#include "nuslam/TurtleMap.h"

#include<random>
 std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

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
      std::normal_distribution<> r_noise(0, 0.017);
      std::normal_distribution<> b_noise(0, 0.01);

      cout<<"success";
      size_t i;
      string objectName="cylinder";
      nuslam::TurtleMap mapMessage;
      mapMessage.header.stamp = ros::Time::now();

      size_t count=0;
      size_t robotPoseIdx;
      for(i=0; i< modelStateMsg.name.size(); i++) 
      {
        if(modelStateMsg.name[i] == "ddrive")
        {
          robotPoseIdx = i;
        }

      }

      auto robotPose = modelStateMsg.pose[robotPoseIdx].position;
      for(i=0; i< modelStateMsg.name.size(); i++)
      {
        if(modelStateMsg.name[i].find(objectName) != std::string::npos)
        {
        mapMessage.landmarkIndex.push_back(count);
        mapMessage.centerX.push_back(modelStateMsg.pose[i].position.x);
        mapMessage.centerY.push_back(modelStateMsg.pose[i].position.y);
        double bearing = atan2(robotPose.y - modelStateMsg.pose[i].position.y,
                             robotPose.x - modelStateMsg.pose[i].position.x);
        double range = sqrt(pow(robotPose.y - modelStateMsg.pose[i].position.y, 2)+
                            pow(robotPose.x - modelStateMsg.pose[i].position.x, 2));
        bearing += r_noise(get_random());
        range += r_noise(get_random());
        mapMessage.range.push_back(range);
        mapMessage.bearing.push_back(bearing);

        mapMessage.radius.push_back(0.04);
        count += 1;
        }
      }
      mapMessage.landmarkCount = count;
      landmarkPublisher.publish(mapMessage);

    }

};



int main(int argc, char** argv)
{
  Analysis analysis(argc, argv);
  ros::spin();
  return 0;
}

