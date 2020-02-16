/// \file
///// \brief This file contains all functions definitions regarding 2d rigid body 
///// transforms. In particular, the 2D homogeneous transformation representations,
///// changing reference frames of points and twists are the core logic in the file
#include"rigid2d/rigid2d.hpp"
#include<exception>
namespace rigid2d
{

  DiffDrive::DiffDrive()
  {
   wheelBase = 5;
   wheelRadius = 3;

  }

  DiffDrive::DiffDrive(Transform2D initialPose, double wheel_base, double wheel_radius)
  {
    currentPose = initialPose;
    wheelBase = wheel_base;
    wheelRadius = wheel_radius;
  }

  WheelVelocities DiffDrive::twistToWheelVelocities(Twist2D BodyTwist, double totalTime)
  { if(BodyTwist.vy > 1e-2)
    {
      throw std::invalid_argument("wrong twist");
    }
    WheelVelocities diffwheels;
    diffwheels.left = (- this->wheelBase/ 2  * BodyTwist.wz + BodyTwist.vx) / this->wheelRadius;
    diffwheels.right = (this->wheelBase /2 * BodyTwist.wz + BodyTwist.vx)/ this->wheelRadius;
    return diffwheels;
  }

  Twist2D DiffDrive::WheelVelocitiestoTwist(WheelVelocities velocities)
  {
    Twist2D BodyTwist;
    BodyTwist.wz = - this->wheelRadius / this->wheelBase * velocities.left + 
                     this->wheelRadius / this->wheelBase * velocities.right;
    BodyTwist.vx = this->wheelRadius / 2 * velocities.left  + this->wheelRadius / 2 * velocities.right;
    BodyTwist.vy = 0;
    return BodyTwist;
  }

  void DiffDrive::UpdateOdometry(double phiLeft, double phiRight)
  {
    if (abs(phiLeft) < 1e-4 && abs(phiRight)<1e-4)
    {
     return void();
    }
    WheelVelocities velocities = {phiLeft, phiRight};
    auto BodyTwist = WheelVelocitiestoTwist(velocities);
    auto T_b_bd = integrateTwist(BodyTwist);
    currentPose = currentPose * T_b_bd;
  }

  void DiffDrive::feedforward(Twist2D BodyTwist, double totalTime)
  {

    auto T_b_bd = integrateTwist(BodyTwist, totalTime);
    currentPose = currentPose * T_b_bd;
  }

  TransformParameters DiffDrive::returnPose() 
  {
    return this->currentPose.displacement();

  }

  WheelVelocities DiffDrive::returnLastEncoderVelocities() const
  {
    return previousWheelVelocities;
  }

  void DiffDrive::reset(Transform2D newPose)
  {
    this->currentPose = newPose;
  } 


  Transform2D::Transform2D(double theta_in, double ctheta_in, double stheta_in, double x_in,
                           double y_in)
{
  theta = theta_in;
  ctheta = ctheta_in;
  stheta = stheta_in;
  x = x_in;
  y = y_in;
}

Vector2D Vector2D::operator+=(const Vector2D & rhsVector)
{
  this->x = this->x + rhsVector.x;
  this->y = this->y + rhsVector.y;
  return *this;

}

Vector2D Vector2D::operator-=(const Vector2D & rhsVector)
{
  this->x = this->x - rhsVector.x;
  this->y = this->y - rhsVector.y;
  return *this;
}

Vector2D Vector2D::operator*=(const double scaling_factor)
{
 this-> x = this-> x * scaling_factor;
 this-> y = this->y * scaling_factor;
 return *this;
}

Vector2D operator+(Vector2D  lhsVector, const Vector2D & rhsVector)
{
  lhsVector += rhsVector;
  return lhsVector;
}

Vector2D operator-(Vector2D  lhsVector, const Vector2D & rhsVector)
{
  lhsVector -= rhsVector;
  return lhsVector;
}

Vector2D operator*(Vector2D  lhsVector, const double scaling_constant)
{

lhsVector *= scaling_constant;
return lhsVector;
}

Vector2D operator*(const double scaling_constant, Vector2D  lhsVector)
{

lhsVector *= scaling_constant;
return lhsVector;
}



AxisAngle Twist2D::return_axis_angle_representation() const
{
  AxisAngle normalised_twist;
  if(this->wz == 0)
  {
   normalised_twist.angle = pow(pow(this->vx, 2) + pow(this->vy, 2), 0.5); 
   normalised_twist.vx = this->vx / normalised_twist.angle;
   normalised_twist.vy = this->vy / normalised_twist.angle;
   normalised_twist.wz = 0;
  }
  else
  {
    normalised_twist.angle = abs(this->wz);
    normalised_twist.vx = this-> vx / normalised_twist.angle;
    normalised_twist.vy = this->vy / normalised_twist.angle;
    if(this->wz > 0)
      normalised_twist.wz = 1.0;
    else
      normalised_twist.wz = -1.0;
  }
  return normalised_twist;

}


Transform2D integrateTwist(const Twist2D& V1, double totalTime) 
{
  double c_theta, s_theta, theta, x, y;
  Twist2D scaledV1 = {V1.wz * totalTime, V1.vx * totalTime, V1.vy * totalTime};
  auto normalisedV1 = scaledV1.return_axis_angle_representation();
  c_theta = -normalisedV1.wz * normalisedV1.wz * (1 - cos(normalisedV1.angle)) + 1;
  s_theta = normalisedV1.wz * sin(normalisedV1.angle);
  theta = atan2(s_theta, c_theta);
  x = normalisedV1.vx * (normalisedV1.angle- (normalisedV1.angle- sin(normalisedV1.angle)) 
      * normalisedV1.wz * normalisedV1.wz) - normalisedV1.vy * (1 -cos(normalisedV1.angle)) * normalisedV1.wz;
  y = normalisedV1.vx * normalisedV1.wz * (1 - cos(normalisedV1.angle))  + 
      normalisedV1.vy * (normalisedV1.angle- (normalisedV1.angle- sin(normalisedV1.angle))
      * normalisedV1.wz * normalisedV1.wz);
  Vector2D t{x, y};
  Transform2D integratedTransform(t, theta);
  return integratedTransform;


}

std::ostream & operator<<(std::ostream & os, const rigid2d::Vector2D & v)
{
  os<<v.x<<"  "<<v.y<<"\n";
  return os;
}

TransformParameters Transform2D::displacement() const
{
  TransformParameters params;
  params.x = this->x;
  params.y = this->y;
  params.theta = this->theta;
  return params;
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
lhs *= rhs;
return lhs;
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
  double old_x = v.x;
  v.x = this->ctheta * v.x - this->stheta * v.y + this-> x;
  v.y = this->stheta * old_x + this->ctheta * v.y + this-> y;
  return v;

}


Twist2D Transform2D::operator()(Twist2D v) const
{
double old_twistvx = v.vx;
v.vx = v.wz * this->y + this->ctheta * v.vx - this->stheta * v.vy;
v.vy = -this->x * v.wz + this-> stheta * old_twistvx + this->ctheta * v.vy;
return v;
}

Transform2D Transform2D::inv() const{
  Transform2D inv_transform;
  inv_transform.theta = atan2(sin(-this->theta), cos(-this->theta));
  inv_transform.ctheta = this->ctheta;
  inv_transform.stheta = -this->stheta;
  inv_transform.x = -this->ctheta * this->x - this->stheta * this->y;
  inv_transform.y = this->stheta * this->x - this-> ctheta * this->y;
  return inv_transform;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{

  this->theta = this->theta + rhs.theta;
  double prev_c_theta = this->ctheta;
  double prev_s_theta = this->stheta;
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
  double theta, x, y;
  is >> theta;
  is >> x;
  is >> y;
  Vector2D temp_vec{x, y};
  Transform2D temp_transform(temp_vec, theta);
  tf = temp_transform;
  return is;

}

Vector2D Vector2D::operator/=(const double divisor)
{
  this->x = this->x / divisor;
  this->y = this->y / divisor;
  return *this;
}


Vector2D operator/(Vector2D v1, const double divisor)
{
  v1 /= divisor;
  return v1;

}


void Vector2D::normalise()
{
  double length = pow(pow(this->x, 2) + pow(this->y, 2), 0.5);
  *this = *this / length;

}

}
