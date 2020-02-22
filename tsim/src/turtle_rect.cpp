#include<turtle_rect.hpp>

using namespace std;

namespace Turtle_Navigation
{
 Circular_Linked_List::Circular_Linked_List(Circular_Linked_List* previous_element, 
                                            int x_value, int y_value)
 {
   x =  x_value;
   y = y_value;
   if(previous_element != NULL)
   {
     next_element = previous_element->next_element;
     previous_element-> next_element = this;
   }
   else
   {
     next_element = this;
   }
 }

Rect_Navigation::Rect_Navigation(int argc, char **argv)
{

  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle n;
  ros::param::get("height", height);
  ros::param::get("width", width);
  ros::param::get("x", x);
  ros::param::get("y", y);
  ros::param::get("frequency", frequency);
  ros::param::get("rot_vel", rot_vel);
  ros::param::get("trans_vel", trans_vel);
  cout<<height<<"\n"<<width<<"\n"<<x<<"\n"<<y<<"\n"<<frequency<<"\n"<<rot_vel<<"\n"<<trans_vel<<"\n";
  ROS_INFO_STREAM("\n height:" <<height<<"\nwidth:" <<width <<"\nx"<<x<<"\ny"<<y
                  <<"\nfrequency:"<<frequency<<"\nrotational velocity"<<rot_vel<<"\ntranslational velocity"<<trans_vel);
  state = INITIALISE;
  current_waypoint = create_waypoints_list();
  //first_waypoint = current_waypoint;
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  reset_turtle = n.advertiseService("/traj_reset", &Rect_Navigation::reset_turtle_callback,
                                     this);
  pose_error_publisher = n.advertise<tsim::ErrorPose>("/pose_error", 1000);
  pose_subscriber = n.subscribe("/turtle1/pose", 1000, &Rect_Navigation::pose_callback,
                                    this);

}

bool Rect_Navigation::reset_turtle_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response)
{
  state = RESET;
  ROS_INFO_STREAM("Inside service\n");
  return true;
}

void Rect_Navigation::pose_callback(const turtlesim::Pose pose)
{
  ROS_INFO_STREAM("pose_x  "<<pose.x<<"  pose_y  "<<pose.y<<"pose theta"<<pose.theta);
  tsim::ErrorPose  error_message;
  error_message.x_error = abs(pose.x - current_x);
  error_message.y_error = abs(pose.y - current_y);
  float angle_diff = pose.theta - current_angle;
  float angle_diff_wrapped = atan2(sin(angle_diff), cos(angle_diff));
  error_message.theta_error= abs(angle_diff_wrapped);
  pose_error_publisher.publish(error_message);

}

Circular_Linked_List* Rect_Navigation::create_waypoints_list()
    {

    Circular_Linked_List* head = new Circular_Linked_List(NULL, x, y);
    //Circular_Linked_List* first_waypoint = new Circular_Linked_List(NULL, x, y);
    first_waypoint = new Circular_Linked_List(head, x+width, y);
    return first_waypoint;
    }


void Rect_Navigation::initialise_turtle()
{

   ros::NodeHandle n;



   ros::service::waitForService("turtle1/set_pen", 5000);
   ros::service::waitForService("turtle1/teleport_absolute", 5000);
   ros::ServiceClient Turtle1_Set_Pen = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
   turtlesim::SetPen pen;
   pen.request.off=1;
   Turtle1_Set_Pen.call(pen);

   ros::ServiceClient Turtle1_Teleport = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
   turtlesim::TeleportAbsolute teleport;
   teleport.request.x = x;
   teleport.request.y = y;
   teleport.request.theta = 0;
   Turtle1_Teleport.call(teleport);

   pen.request.r = 255;
   pen.request.g = 0;
   pen.request.b = 0;
   pen.request.width = 5;
   pen.request.off=0;
   Turtle1_Set_Pen.call(pen);
   state = FOLLOW_WAYPOINT;

   goal_angle = 0;
   current_angle = 0;
   current_x = x;
   current_y = y;
   previous_time = ros::Time::now();

}

void Rect_Navigation::update_current_pose()
{
  ros::Duration time_duration = ros::Time::now() - previous_time;
  double time_elapsed = time_duration.toSec();
  ROS_INFO_STREAM("time elapsed:  "<<time_elapsed);
  current_angle += time_elapsed * previous_velocity.angular.z;
  current_x += time_elapsed * previous_velocity.linear.x * cos(current_angle);
  current_y += time_elapsed * previous_velocity.linear.x * sin(current_angle);
  ROS_INFO_STREAM("current_x:  " << current_x <<"current_y:  " <<current_y<<"current theta:  "<<current_angle);
  current_angle = atan2(sin(current_angle), cos(current_angle));
  ROS_INFO_STREAM("finished update pose");
}

void Rect_Navigation::follow_way_point()
{
  geometry_msgs::Twist velocity_message;
  update_current_pose();

  ROS_INFO_STREAM("accessing waypoint");
  float goal_angle = atan2(current_waypoint->y - current_y, current_waypoint->x - current_x);
  ROS_INFO_STREAM("finished accessing waypoint");
  float angle_difference = current_angle - goal_angle;
  float angle_diff_wrapped = atan2(sin(angle_difference), cos(angle_difference));
  float distance_difference = pow(pow((current_waypoint->x - current_x), 2) +
                              pow((current_waypoint->y - current_y), 2), 0.5);


  if(abs(angle_diff_wrapped) >= 0.1)
  {
    ROS_INFO_STREAM("angle difference" << current_angle - goal_angle);
     if (angle_diff_wrapped > 0)
       velocity_message.angular.z =  -rot_vel;
     else
      velocity_message.angular.z =  rot_vel;
    velocity_publisher.publish(velocity_message);
  } 
  else if (distance_difference > 0.1)
  {

    ROS_INFO_STREAM("distance difference" <<pow(pow((current_waypoint->x - current_x), 2) +
                                             pow((current_waypoint->y - current_y), 2), 0.5));
    velocity_message.linear.x = trans_vel;
    velocity_publisher.publish(velocity_message);
  } 
  else
  {
    ROS_INFO_STREAM("changing way point");
    state = CHANGE_WAY_POINT;
  }
  previous_time = ros::Time::now();
  previous_velocity = velocity_message;
  ROS_INFO_STREAM("finished one cycle of following way point");
}


void Rect_Navigation::execute_trajectory()
{
  switch(state)
  {
  case INITIALISE      :initialise_turtle();
                        break;
  case FOLLOW_WAYPOINT :follow_way_point();
                        break;
  case CHANGE_WAY_POINT:current_waypoint = current_waypoint->next_element;
                        state = FOLLOW_WAYPOINT;
                        break;
  case RESET: ROS_INFO_STREAM("started_reset");
              initialise_turtle();
              current_waypoint = first_waypoint;
              current_x = x;
              current_y = y;
              current_angle = 0;
              state = FOLLOW_WAYPOINT;
              ROS_INFO_STREAM("Finished reset");
              break;

}

}
}

int main(int argc, char **argv){
  Turtle_Navigation::Rect_Navigation Rect_Turtle(argc, argv);
  ros::Rate loop_rate(Rect_Turtle.frequency);
  while (ros::ok())
  {
  Rect_Turtle.execute_trajectory();
  loop_rate.sleep();
  ros::spinOnce();
  }

  return 0;
}




