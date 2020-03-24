/// \file
/// \brief A file for detecting circle using a regression algorithm. This file is a library file
///


#include "sensor_msgs/LaserScan.h"
#include "nuslam/circle_detection.hpp"
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <cmath>
#include "rigid2d/rigid2d.hpp"


using std::cout;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::BDCSVD;
using Eigen::ComputeFullV;
using Eigen::ComputeFullU;


using Eigen::Vector4d;
using rigid2d::Vector2D;

typedef Matrix<double, Dynamic, 4> Matrix4dn;
typedef Matrix<double, 4, 4> Matrix4d;



namespace circleDetection
{

/// \brief A constructor for Poin2d data type
/// \param x1 - x co-ordinate
/// \param y1 - y co-ordinate
Point2d::Point2d(double x1, double y1)
{

  x = x1;
  y = y1;
}

/// \brief A function that fits circle to the list of points given
/// \param CirclePoints -  A list containing points for which a circle needs to be fit.
Vector3d fitCircle(vector<Vector2D> circlePoints)
{

  Matrix4dn circleMatrix;
  Matrix4d H, Hinv;
  circleMatrix.resize(circlePoints.size(), 4);
  Vector4d equCoeff;
  size_t i;
  double sum_x=0, sum_y=0, sum_z=0;
  int pointsCount = circlePoints.size();

  for(auto point: circlePoints)
  {
    sum_x += point.x;
    sum_y += point.y;
  }
  double mean_x = sum_x / (double) circlePoints.size();
  double mean_y = sum_y / (double) circlePoints.size();

  for (auto& point:circlePoints)
  {
    point.x = point.x - mean_x;
    point.y = point.y - mean_y;
  }

  std::cout << "Here is the matrix m:\n" << circlePoints[0].x <<circlePoints[0].y << std::endl;


  for(i=0; i < circlePoints.size(); i++)
  {
    circleMatrix(i, 0) = pow(circlePoints[i].x, 2) + pow(circlePoints[i].y, 2);
    circleMatrix(i, 1) = circlePoints[i].x;
    circleMatrix(i, 2) = circlePoints[i].y;
    circleMatrix(i, 3) = 1.0;
    sum_z += circleMatrix(i, 0);
  }

  double mean_z = sum_z / (double) pointsCount;

  auto M = circleMatrix.transpose() * circleMatrix / pointsCount;

  // Initialise H matrix
  H(0,0) = 8 * mean_z;
  H(0,1) = 0;
  H(0,2) = 0;
  H(0,3) = 2;
  H(1,0) = 0;
  H(1,1) = 1;
  H(1,2) = 0;
  H(1,3) = 0;
  H(2,0) = 0;
  H(2,1) = 0;
  H(2,2) = 1;
  H(2,3) = 0;
  H(3,0) = 2;
  H(3,1) = 0;
  H(3,2) = 0;
  H(3,3) = 0;


  // Initialise H inverse matrix
  Hinv(0,0) = 0;
  Hinv(0,1) = 0;
  Hinv(0,2) = 0;
  Hinv(0,3) = 0.5;
  Hinv(1,0) = 0;
  Hinv(1,1) = 1;
  Hinv(1,2) = 0;
  Hinv(1,3) = 0;
  Hinv(2,0) = 0;
  Hinv(2,1) = 0;
  Hinv(2,2) = 1;
  Hinv(2,3) = 0;
  Hinv(3,0) = 0.5;
  Hinv(3,1) = 0;
  Hinv(3,2) = 0;
  Hinv(3,3) = -2 * mean_z;

  BDCSVD<Matrix4dn> svdCircle( circleMatrix, ComputeFullV | ComputeFullU  );
  auto sigmaValues = svdCircle.singularValues();
  auto V = svdCircle.matrixV();


  if(sigmaValues.size() ==4 && sigmaValues[3] > 1e-12)
  {

    auto sigmaMatrix = sigmaValues.asDiagonal();
    auto Y = V * sigmaMatrix * V.transpose();
    std::cout<< "Y cols" << Y.cols() <<" Y rows"<<Y.rows();
    std::cout<< "H cols" << Hinv.cols() <<" H rows" << Hinv.rows();

    std::cout << "Here is the matrix m:\n" << Y << std::endl;
    auto Q = Y * Hinv * Y;
    Eigen::SelfAdjointEigenSolver<Matrix4dn> eig(Q);

    auto eigVecQ = eig.eigenvectors();
    auto eigValues = eig.eigenvalues();

    double minEigValue = std::numeric_limits<double>::infinity();
    int minEigIndex;

    for(int i=0; i< eigValues.size(); i++)
    {
      if(eigValues[i] < minEigValue && eigValues[i] > 0)
      {
        minEigValue = eigValues[i];
        minEigIndex = i;
      }
    }

    Vector4d smallEigVec = {eigVecQ(0,minEigIndex), eigVecQ(1,minEigIndex), eigVecQ(2,minEigIndex), eigVecQ(3,minEigIndex)};

    std::cout << "Here is the matrix m:\n" << eigVecQ << std::endl;
    std::cout << "Here is the matrix m:\n" << smallEigVec << std::endl;
    std::cout << "Q eigen values" << eigValues;

    equCoeff = Y.colPivHouseholderQr().solve(smallEigVec);
  }
  else
  {

    equCoeff = {V(0,3), V(1,3), V(2,3), V(3,3)};
  }

  double a = -equCoeff[1] / equCoeff[0] / 2.0 + mean_x;
  double b = -equCoeff[2] / equCoeff[0] / 2.0 + mean_y;
  double r2 = (pow(equCoeff[1], 2) + pow(equCoeff[2], 2) - 4 * equCoeff[0] * equCoeff[3]) / (4.0 * pow(equCoeff[0], 2));
  double r = sqrt(r2);
  Vector3d circleCoeff = {a, b, r};




 std::cout << "Here is the matrix m:\n" << circleMatrix << std::endl;
 std::cout << "Here is the matrix m:\n" << M << std::endl;
 std::cout<< "Sigma values\n"<<sigmaValues << std::endl;
 std::cout<< "Circle coefficients" << circleCoeff;
 std::cout<< "x center" << mean_x;
 std::cout<< "y center" << mean_y;
 return circleCoeff;


}

/// \brief A function that calculates the least squares error for the estimated circle
///        fitting parameters
/// \param observedPoints - All the points for which a circular fit needs to be done
/// \param circleCoeff - Circle parameters estimated

double calculateError(const vector<Vector2D>& observedPoints, Vector3d circleCoeff)
{
  double totalError=0;
  double error = 0;

  for(auto point:observedPoints)
  {
    error = pow(point.x - circleCoeff[0], 2) + pow(point.y - circleCoeff[1], 2) - pow(circleCoeff[2], 2);
    error = pow(error, 2);
    totalError += error;
  }
  totalError = totalError / (double) observedPoints.size();
  totalError = sqrt(totalError);
  return totalError;

}


}




