#include<iostream>
#include<ros/ros.h>
#include<ros/console.h>
#include<geometry_msgs/Twist.h> 
#include<turtlesim/SetPen.h>
#include<turtlesim/TeleportAbsolute.h>

namespace Turtle_Navigation
{
  class Rect_Navigation
  {
    public:
      Rect_Navigation(int argc, char **argv);
      void restart_trajectory();
      void pose_callback();
      void execute_trajectory();
    private:
      int height, width, x, y;
      float frequency, rot_vel, trans_vel;
      void initialise_turtle();
  };

}
