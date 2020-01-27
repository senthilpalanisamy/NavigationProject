/// \file
///// \brief This file contains all functions definitions regarding waypoint node 
///// transforms. In particular, it contains the constructor for waypoint class and 
///// next velocity function that specifies the next velocity for following a waypoint
///// Many other function definitions, particularly the circular linked list implementation
///// lie as templates in the hpp file
#include"rigid2d/waypoint.hpp"

namespace rigid2d
{


  Waypoint::Waypoint(std::vector<Vector2D> PointSequence)
  {
    trajectoryPoints.addElements(PointSequence);
    currentWaypoint = trajectoryPoints.returnNextElement();
    maxLinearVelocity = 0.5;
    maxAngularVelocity = 0.5;
  }


  WheelVelocities Waypoint::nextVelocity(float timeInterval)
  {


  auto currentPose = diffCar.returnPose();
  float goal_angle = atan2( currentWaypoint.y - currentPose.y,
                            currentWaypoint.x - currentPose.x);
  float angle_difference = currentPose.theta - goal_angle;
  float angle_diff_wrapped = atan2(sin(angle_difference), cos(angle_difference));
  float distance_difference = pow(pow((currentWaypoint.x - currentPose.x), 2) +
                              pow((currentWaypoint.y - currentPose.y), 2), 0.5);


  Twist2D twist2BeApplied;
  WheelVelocities velocities;
  if(abs(angle_diff_wrapped) >= 0.1)
  {
     if (angle_diff_wrapped > 0)
       twist2BeApplied.wz =  -maxAngularVelocity;
     else
       twist2BeApplied.wz  =  maxAngularVelocity;
  } 
  else if(distance_difference > 0.1)
  {

    twist2BeApplied.vx = maxLinearVelocity;
  }

  if(abs(angle_diff_wrapped) >= 0.1 or distance_difference > 0.1)
  {
    velocities = diffCar.twistToWheelVelocities(twist2BeApplied);
  }
  else
  {
    currentWaypoint = trajectoryPoints.returnNextElement();
    velocities = nextVelocity(timeInterval);
  }
  twist2BeApplied.wz = twist2BeApplied.wz * timeInterval;
  twist2BeApplied.vx = twist2BeApplied.vx * timeInterval;
  twist2BeApplied.vy = twist2BeApplied.vy * timeInterval;
  diffCar.feedforward(twist2BeApplied);
  return velocities;
  }



}

