#ifndef CIRCLE_DETECTION_INCLUDE_GAURD
#define CIRCLE_DETECTION_INCLUDE_GAURD
#include <Eigen/Dense>
#include <vector>

using std::vector;
using Eigen::Vector3d;

namespace circleDetection
{
struct Point2d
{
  Point2d(double x, double y);
  double x, y;
};


Vector3d fitCircle(vector<Point2d> circlePoints);
double calculateError(const vector<Point2d>& observedPoints, Vector3d circleCoeff);
}



#endif
