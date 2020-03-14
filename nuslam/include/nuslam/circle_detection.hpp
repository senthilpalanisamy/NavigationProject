#ifndef CIRCLE_DETECTION_INCLUDE_GAURD
#define CIRCLE_DETECTION_INCLUDE_GAURD
#include <Eigen/Dense>
#include <vector>
#include "rigid2d/rigid2d.hpp"

using std::vector;
using Eigen::Vector3d;
using rigid2d::Vector2D;

namespace circleDetection
{
struct Point2d
{
  Point2d(double x, double y);
  double x, y;
};


Vector3d fitCircle(vector<Vector2D> circlePoints);
double calculateError(const vector<Vector2D>& observedPoints, Vector3d circleCoeff);
}



#endif
