#ifndef _TURTLEDRIVE_PLUGIN_HH_
#define _TURTLEDRIVE_PLUGIN_HH_

#include<iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using std::cout;
using std::string;

namespace gazebo
{
  class TurtleDrivePlugin : public ModelPlugin
  {
    physics::ModelPtr model;
    string left_joint, right_joint;
    double sensor_frequency;
    int encoder_ticks_per_rev;
    public: TurtleDrivePlugin (): ModelPlugin ()
      {
        std::cerr<<"Initialised";
        printf("Initialised\n");
      }

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {

        this->model = _model;
        string left_wheel_joint, right_wheel_joint;
        if(! _sdf-> HasElement("left_wheel_joint"))
         {
           std::cerr <<"left joint not specified";

         }
         else
         {
         left_joint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
          std::cerr<<"left wheel joint name is"<<left_wheel_joint;

         }


        if(! _sdf-> HasElement("right_wheel_joint"))
         {
           std::cerr <<"right joint not specified";

         }
         else
         {
         right_joint =_sdf->GetElement("right_wheel_joint")->Get<std::string>();
          std::cerr<<"right wheel joint name is"<<left_wheel_joint;
         }
         
         if(! _sdf-> HasElement("sensor_frequency"))
         {
           sensor_frequency = std::stod(_sdf->GetElement("sensor_frequency")->Get<std::string>());
         }
         else
         {
           sensor_frequency = 200.0;
         }

         if(! _sdf-> HasElement("encoder_ticks_per_rev"))
         {

           std::cerr <<"encoder ticks per revolution not present";
         }
         else
         {

           encoder_ticks_per_rev = std::stoi(_sdf->GetElement("encoder_ticks_per_rev")->Get<std::string>());

         }

        std::cerr<<"sensor frequency"<<sensor_frequency;



        std::cerr<<"load function";
        //gzdbg << "message" << std::endl;

      }
  };
  GZ_REGISTER_MODEL_PLUGIN(TurtleDrivePlugin);
}
#endif
