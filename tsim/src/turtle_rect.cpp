#include<turtle_rect.hpp>

using namespace std;

namespace Turtle_Navigation
{

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
  ros::Rate loop_rate(frequency);
  initialise_turtle();
  // ros::Rate loop_rate(10.0);

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
   Turtle1_Set_Pen.call(pen);

}

}

int main(int argc, char **argv){
  Turtle_Navigation::Rect_Navigation Rect_Turtle(argc, argv);

  //ros::init(argc, argv, "turtle_rect");
  //int height, width, x, y;
  //float frequency, rot_vel, trans_vel;

  // ros::param::get("height", height);
  // ros::param::get("width", width);
  // ros::param::get("x", x);
  // ros::param::get("y", y);
  // ros::param::get("frequency", frequency);
  // ros::param::get("rot_vel", rot_vel);
  // ros::param::get("trans_vel", trans_vel);
  // cout<<height<<"\n"<<width<<"\n"<<x<<"\n"<<y<<"\n"<<frequency<<"\n"<<rot_vel<<"\n"<<trans_vel<<"\n";
  // ROS_INFO_STREAM("\n height:" <<height<<"\nwidth:" <<width <<"\nx"<<x<<"\ny"<<y
  //                 <<"\nfrequency:"<<frequency<<"\nrotational velocity"<<rot_vel<<"\ntranslational velocity"<<trans_vel);
  // ros::Rate loop_rate(frequency);
  return 0;
}




