#include"rigid2d.hpp"
//using namespace rigid2d;

namespace rigid2d
{
std::ostream & operator<<(std::ostream & os, const rigid2d::Vector2D & v)
{
  os<<v.x<<"  "<<v.y<<"\n";
  return os;
}


std::istream & operator>>(std::istream & is, Vector2D & v)
{
  is >> v.x;
  is >> v.y;
  return is;

}
}
