
#include<iostream>
#include"rigid2d.hpp"
#include <vector>


using namespace std;
// using namespace rigid2d;

int main(void)
{

  rigid2d::Transform2D Tab, Tbc, Tac, Tca, Tba, Tcb;
  rigid2d::Vector2D v;
  char frame;

  cout<<"\nEnter Transformation Tab: order - dtheta, dx, dy:";
  cin>> Tab;


  cout<<"\nEnter Transformation Tbc: order - dtheta, dx, dy:";
  cin>> Tbc;

  Tac *= Tab;
  Tac *= Tbc;
  Tca = Tac.inv();
  Tba = Tab.inv();
  Tcb = Tbc.inv();

  cout<<"\nEnter the vector x,y:\n";
  cin>>v;
  cout<<"\nEnter the frame:";
  cin>>frame;
  vector<rigid2d::Vector2D> all_points;
  char frame_ids[3] = {'a', 'b', 'c'};
  switch(frame)
  {
    case 'a': all_points.push_back(v);
              all_points.push_back(Tba(v));
              all_points.push_back(Tca(v));
              break;
    case 'b': all_points.push_back(Tab(v));
              all_points.push_back(v);
              all_points.push_back(Tac(v));
              break;
    case 'c': all_points.push_back(Tac(v));
              all_points.push_back(Tbc(v));
              all_points.push_back(v);
  }

  for(int i=0; i<=2; i++)
  {
    cout<<"point in frame  "<<frame_ids[i]<<"  is  :"<<all_points[i]<<"\n";
  }



 }
