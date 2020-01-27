/// \file
///// \brief This file is just a way to interface with the core rigid2d library
///// and test the implementation. In particular, this file accepts inputs for transforms
///// points and twists in any user defined frame, and converts them those points 
///// and frames to all other existing frames and displays them
#include<iostream>
#include"rigid2d/rigid2d.hpp"
#include <vector>


using namespace std;

int main(void)
{

  rigid2d::Transform2D Tab, Tbc, Tac, Tca, Tba, Tcb;
  rigid2d::Vector2D v;
  char frame;
  rigid2d::Twist2D Tw;



  cout<<"\nEnterrrr Transformation Tab: order - dtheta, dx, dy:";
  cin>> Tab;

  cout<<"\nPrinting transformation\n";
  cout<<Tab;


  cout<<"\nEnter Transformation Tbc: order - dtheta, dx, dy:";
  cin>> Tbc;
  Tac = Tab * Tbc;
  Tca = Tac.inv();
  Tba = Tab.inv();
  Tcb = Tbc.inv();


  cout<<"\nEnter the frame for vector and twist:";
  cin>>frame;
  cout<<"\nEnter the vector x,y:\n";
  cin>>v;

  cout<<"\nEnter a Twist: order wz vx vy:";
  cin>>Tw;
  vector<rigid2d::Vector2D> all_points;
  vector<rigid2d::Twist2D> all_twists;
  char frame_ids[3] = {'a', 'b', 'c'};
  switch(frame)
  {
    case 'a': all_points.push_back(v);
              all_points.push_back(Tba(v));
              all_points.push_back(Tca(v));
              all_twists.push_back(Tw);
              all_twists.push_back(Tba(Tw));
              all_twists.push_back(Tca(Tw));
              break;
    case 'b': all_points.push_back(Tab(v));
              all_points.push_back(v);
              all_points.push_back(Tac(v));
              all_twists.push_back(Tab(Tw));
              all_twists.push_back(Tw);
              all_twists.push_back(Tca(Tw));
              break;
    case 'c': all_points.push_back(Tac(v));
              all_points.push_back(Tbc(v));
              all_points.push_back(v);
              all_twists.push_back(Tac(Tw));
              all_twists.push_back(Tbc(Tw));
              all_twists.push_back(Tw);
  }

  for(int i=0; i<=2; i++)
  {
    cout<<"point in frame  "<<frame_ids[i]<<"  is  :"<<all_points[i]<<"\n";
    cout<<"twist in frame  "<<frame_ids[i]<<"  is  :"<<all_twists[i]<<"\n";
  }



 }
