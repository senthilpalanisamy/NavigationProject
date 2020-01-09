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

Transform2D::Transform2D()
    {
      theta = 0.0;
      ctheta = 1.0;
      stheta = 0.0;
      x = 0.0;
      y = 0.0;
    }
    
Transform2D::Transform2D(const Vector2D & trans)
{
      theta = 0.0;
      ctheta = 1.0;
      stheta = 0.0;
      x = trans.x;
      y = trans.y;

}

Transform2D::Transform2D(double radians)
{
      theta = radians;
      ctheta = cos(rad2deg(radians));
      stheta = sin(rad2deg(radians));
      x = 0;
      y = 0;
}

Transform2D::Transform2D(const Vector2D & trans, double radians)
{
      theta = radians;
      ctheta = cos(rad2deg(radians));
      stheta = sin(rad2deg(radians));
      x = trans.x;
      y = trans.y;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
  v.x = this->ctheta * v.x +- this->stheta * v.y + this-> x;
  v.y = this->stheta * v.x + this->ctheta * v.y + this-> y;
  return v;

}

Transform2D Transform2D::inv() const{
  Transform2D inv_transform;
  inv_transform.theta = -this->theta;
  inv_transform.ctheta = this->ctheta;
  inv_transform.stheta = -this->stheta;
  inv_transform.x = -this->ctheta * this->x - this->stheta * this->y;
  inv_transform.y = this->stheta * this->x + this-> ctheta * this->y;
  return inv_transform;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  this->ctheta = this->ctheta * rhs.ctheta - this->stheta * rhs.stheta;
  this->stheta = this->ctheta * rhs.stheta + this->stheta * rhs.ctheta;
  this->x = this->ctheta * rhs.x - this->stheta * rhs.y + this->x;
  this->y = this->stheta * rhs.x + this-> ctheta * rhs.y + this->y;
  this->theta = this->theta + rhs.theta;
  return *this;
}

std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  os<<"dtheta:"<<tf.theta<<"  dx:"<<tf.x<<"  dy:"<<tf.y;
  return os;
}

}
