#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

double clampValue(double input, double upperLimit, double lowerLimit)
{
  double clampedValue;

  if(input > 0)
    {
      clampedValue = std::min(input, upperLimit)
    }
  else
    {
      clampedValue = std::max(input, lowerLimit)
    }
  return clampedValue;

}

class turtleInterface
{

  ros::Subscriber cmdVelSubscriber;
  ros::Publisher wheelCommandPublisher;
  double maxMotorVelocity, robotMaxTransVel, robotMaxRotVel;
  double wheeelBase, wheeelRadius;
  rigid2d::DiffDrive diffcar;
  double maxWheelCommand, minWheelCommand, scaleMotorVelToCmd;
  
  turtleInterface()
  {
  cmdVelSubscriber = n.subscribe("cmd_vel", 1000, &turtleInterface::cmdVelCallback,
                                  this);

  wheelCommandPublisher = n.advertise<nuturtlebot::WheelCommands>("wheel_commands", 1000);

  ros::param::get("max_rot_vel_motor", maxMotorVelocity);
  ros::param::get("max_trans_vel", robotMaxTransVel);
  ros::param::get("max_rot_vel_motor", robotMaxRotVel);
  ros::param::get("wheel_base", wheelBase);
  ros::param::get("wheel_radius", wheelRadius);

  rigid2d::Transform2D identityTransform(0);
  diffcar = rigid2d::DiffDrive(identityTransform, wheelBase, wheelRadius); 
  maxWheelCommand = 44;
  minWheelCommand = -44;
  scaleVelToCmd = (maxWheelCommand - minWheelCommand) / (2  * maxMotorVelocity);
  }

void cmdVelCallback(const geometry_msgs::Twist bodyTwistMsg)

  {
    auto linearVelocity = clampValue(bodyTwistMsg.linear.x, robotMaxTransVel, -robotMaxTransVel);
    auto angularVelocity = clampedValue(bodyTwistMsg.angular.z, robotMaxRotVel, -robotMaxRotVel);

    rigid2d::Twist2D twistFollowed;
    twistFollowed.wz = bodyTwistMsg.angular.z;
    twistFollowed.vx = bodyTwistMsg.linear.x;
    twistFollowed.vy = bodyTwistMsg.linear.y;

    auto velocities = diffcar.twistToWheelVelocities(twistFollowed);
    velocities.left = clampValue(velocities.left, maxMotorVelocity, -maxMotorVelocity);
    velocities.right = clampValue(velocities.right, maxMotorVelocity, -maxMotorVelocity);
    nuturtlebot::WheelCommands wheelVelCommands;
    wheelVelCommands.left_velocity = velocities.left * scaleVelToCmd;
    wheelVelCommands.right_velocity = velocities.right * scaleVelToCmd;
    wheelCommandPublisher.pub(wheelVelCommands);
}
};


int main()
{
  turtleInterface turtleBot;
  ros::spin();
  return 0;
}
