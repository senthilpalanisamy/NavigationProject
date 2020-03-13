#include "nuslam/circle_detection.hpp"
#include <gtest/gtest.h>
#include <vector>

using circleDetection::fitCircle;
using circleDetection::calculateError;
using circleDetection::Point2d;

 TEST(CircleDetection, fitCircle)
 {
   vector<Point2d> observedPoints1;
   observedPoints1.push_back(Point2d(1,7));
   observedPoints1.push_back(Point2d(2,6));
   observedPoints1.push_back(Point2d(5,8));
   observedPoints1.push_back(Point2d(7,7));
   observedPoints1.push_back(Point2d(9,5));
   observedPoints1.push_back(Point2d(3,7));

   auto circleCoeff = fitCircle(observedPoints1);
   auto fittingError = calculateError(observedPoints1, circleCoeff);

  ASSERT_NEAR(circleCoeff[0], 4.615482, 0.0001) <<"Fitting circle failed. TESTCASE:1 Wrong x coordinate calculated";
  ASSERT_NEAR(circleCoeff[1], 2.807354, 0.0001) <<"Fitting circle failed. TESTCASE:1 Wrong y coordinate calculated";
  ASSERT_NEAR(circleCoeff[2], 4.8275, 0.0001) <<"Fitting circle failed. TESTCASE:1 Wrong radius calculated";


   vector<Point2d> observedPoints2;
   observedPoints2.push_back(Point2d(-1, 0));
   observedPoints2.push_back(Point2d(-0.3, -0.06));
   observedPoints2.push_back(Point2d(0.3, 0.1));
   observedPoints2.push_back(Point2d(1, 0));

   circleCoeff = fitCircle(observedPoints2);
   fittingError = calculateError(observedPoints2, circleCoeff);


  ASSERT_NEAR(circleCoeff[0], 0.4908357 , 0.0001) <<"Fitting circle failed. TESTCASE_2 Wrong x coordinate calculated";
  ASSERT_NEAR(circleCoeff[1], -22.15212 , 0.0001) <<"Fitting circle failed. TEST_CASE_2 Wrong y coordinate calculated";
  ASSERT_NEAR(circleCoeff[2], 22.17979 , 0.0001) <<"Fitting circle failed. TEST_CASE_2 Wrong radius calculated";


 }
