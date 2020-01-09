#include<iostream>
#include"rigid2d.hpp"

using namespace std;
using namespace rigid2d;

int main(void)
{
  cout<<true<<"\n";
  cout<<almost_equal(1.0, 1.0)<<"\n";
  cout<<deg2rad(1.0)<<"\n";
  cout<<1.0 * rigid2d::PI / 180.0;
  cout<<rad2deg(1.0)<<"\n";

}
