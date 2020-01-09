#include<iostream>
#include"rigid2d.hpp"

using namespace std;
// using namespace rigid2d;

int main(void)
{
  cout<<"started";
  cout<<true<<"\n";
  cout<<rigid2d::almost_equal(1.0, 1.0)<<"\n";
  cout<<rigid2d::deg2rad(1.0)<<"\n";
  cout<<1.0 * rigid2d::PI / 180.0;
  cout<<rigid2d::rad2deg(1.0)<<"\n";
  rigid2d::Vector2D v;
  v.x = 2.0;
  v.y = 4.0;
  //std::cout<<"here\n";
  cout<<v;
  cout<<"Enter input\n";
  cin>>v;
  cout<<"printing v\n";
  cout<<v;
  rigid2d::Transform2D Tab; 
  rigid2d::Transform2D Tbd; 
  cout<<Tab<<"\n";
  rigid2d::Transform2D Tbc(1.7); 
  cout<<Tbc<<"\n";
  rigid2d::Transform2D Tcd(rigid2d::Vector2D{1.0, 2.0}); 
  //cout<<Tcd<<"\n";
  Tbd *= Tbc;
  Tbd *= Tcd;
  cout<<Tbd;
}
