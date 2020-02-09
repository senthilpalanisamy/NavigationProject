#include <ros/ros.h>
#include <ros/console.h>
#include <nuturtle_robot/StartRotation.h>
#include <rigid2d/SetPose.h> 

class RotateInPlace
{
  ros::ServiceServer startRotation;
  ros::ServiceClient setPose;
  bool isClockwise;
  public:
  RotateInPlace(int argc, char** argv)
  {

    ros::init(argc, argv, "rotate_in_place");
    ros::NodeHandle n;

    startRotation = n.advertiseService("/start", &RotateInPlace::startCallback,
                                     this);
    setPose = n.serviceClient<rigid2d::SetPose>("/set_pose");
    isClockwise = true;
  }

  bool startCallback(nuturtle_robot::StartRotation::Request& request,
                     nuturtle_robot::StartRotation::Response& response)
  {
    if(setPose.exists())
    {
      rigid2d::SetPose initialPose;
      initialPose.request.desiredPose.x = 0;
      initialPose.request.desiredPose.y = 0;
      initialPose.request.desiredPose.theta = 0;
      setPose.call(initialPose);
    }
    isClockwise = request.isClockwise;
    return true;

  }

};



int main(int argc, char** argv )
{
  RotateInPlace turtleRotate(argc, argv);
  ros::spin();
  return 0;
}
