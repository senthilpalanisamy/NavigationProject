/// \file
///// \brief This file contains all functions definitions regarding 2d rigid body 
///// transforms. In particular, the 2D homogeneous transformation representations,
///// changing reference frames of points and twists are the core logic in the file
#include"rigid2d.hpp"

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

std::ostream & operator<<(std::ostream & os, const Twist2D & v)
{

 os<<v.wz<<"  "<<v.vx<<"  "<<v.vy<<"\n"; 
 return os;
}


std::istream & operator>>(std::istream & is, Twist2D & v)
{
  is>>v.wz;
  is>>v.vx;
  is>>v.vy;
  return is;

}


Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
Transform2D result;
result *= lhs;
result *= rhs;
return result;
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
      ctheta = cos(radians);
      stheta = sin(radians);
      x = 0;
      y = 0;
}

Transform2D::Transform2D(const Vector2D & trans, double radians)
{
      theta = radians;
      ctheta = cos(radians);
      stheta = sin(radians);
      x = trans.x;
      y = trans.y;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
  float old_x = v.x;
  v.x = this->ctheta * v.x - this->stheta * v.y + this-> x;
  v.y = this->stheta * old_x + this->ctheta * v.y + this-> y;
  return v;

}


Twist2D Transform2D::operator()(Twist2D v) const
{
float old_twistvx = v.vx;
v.vx = v.wz * this->y + this->ctheta * v.vx - this->stheta * v.vy;
v.vy = -this->x * v.wz + this-> stheta * old_twistvx + this->ctheta * v.vy;
return v;
}

Transform2D Transform2D::inv() const{
  Transform2D inv_transform;
  inv_transform.theta = -this->theta;
  inv_transform.ctheta = this->ctheta;
  inv_transform.stheta = -this->stheta;
  inv_transform.x = -this->ctheta * this->x - this->stheta * this->y;
  inv_transform.y = this->stheta * this->x - this-> ctheta * this->y;
  return inv_transform;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{

  this->theta = this->theta + rhs.theta;
  float prev_c_theta = this->ctheta;
  float prev_s_theta = this->stheta;
  this->ctheta = cos(this->theta);
  this->stheta = sin(this->theta);
  this->x = prev_c_theta * rhs.x - prev_s_theta * rhs.y + this->x;
  this->y = prev_s_theta * rhs.x + prev_c_theta * rhs.y + this->y;
  return *this;
}

std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  os<<"dtheta:"<<tf.theta<<"  dx:"<<tf.x<<"  dy:"<<tf.y;
  return os;
}

std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  is >> tf.theta;
  is>>tf.x;
  is>>tf.y;
  tf.ctheta =  cos(tf.theta);
  tf.stheta =  sin(tf.theta);
  return is;

}


Vector2D Vector2D::operator/(const float divisor)
{
  struct Vector2D divided_vec = {0.0, 0.0};
  divided_vec.x = this->x / divisor;
  divided_vec.y = this->y / divisor;
  return divided_vec;

}

void Vector2D::normalise()
{
  float length = pow(pow(this->x, 2) + pow(this->y, 2), 0.5);
  *this = *this / length;

}

}
