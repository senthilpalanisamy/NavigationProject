#include<iostream>
#include<ros/ros.h>
#include<ros/console.h>
#include<cmath>
#include<geometry_msgs/Twist.h> 
#include<turtlesim/SetPen.h>
#include<turtlesim/TeleportAbsolute.h>
#include <std_srvs/Empty.h>

namespace Turtle_Navigation
{
  enum Turtle_States {INITIALISE, FOLLOW_WAYPOINT, CHANGE_WAY_POINT,
                      RESET};

  struct Circular_Linked_List
  {
    Circular_Linked_List(Circular_Linked_List* previous_element, int x, int y);
    int x, y;
    Circular_Linked_List* next_element;
  };

  class Rect_Navigation
  {
    public:
      Rect_Navigation(int argc, char **argv);
      void restart_trajectory();
      void pose_callback();
      void execute_trajectory();
      void follow_way_point();
      Circular_Linked_List *current_waypoint, *first_waypoint;
      //Circular_Linked_List current_element;
      Turtle_States state;
      double current_x, current_y, goal_angle, current_angle;
      double total_time;
      float frequency;
      geometry_msgs::Twist previous_velocity;
      ros::Time previous_time;
      bool reset_turtle_callback(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response);

      ros::ServiceServer reset_turtle;
      //float x, y;
    private:
      float height, width, x, y;
      //float height, width;
      float rot_vel, trans_vel;
      void initialise_turtle();
      void update_current_pose();
      //ros::NodeHandle n;
      Circular_Linked_List* create_waypoints_list();
      ros::Publisher velocity_publisher;
  };

}
