#include<gtest/gtest.h>
#include"rigid2d/rigid2d.hpp"
#include<sstream>
#include"rigid2d/waypoint.hpp"
/// \file
/// Test cases for unit testing open apis of the rigid2d library

TEST(RigidTransform2D, multiplyTransforms)
{

  rigid2d::Vector2D t_ab = {1.0, 2.0}, t_bc = {4.0, -2.0};
  double theta_ab = 1.0, theta_bc = -2.14;
  rigid2d::Transform2D Tab(t_ab, theta_ab), Tbc(t_bc, theta_bc), answer;
  answer = Tab * Tbc;
  auto Tac_params = rigid2d::TransformParameters{-1.14, 4.84415119,4.28527933};
  rigid2d::TransformParameters answer_values = answer.displacement();

  ASSERT_NEAR(Tac_params.theta, answer_values.theta, 0.01) <<" Multiplication of two transforms using * failed: Value"
                                                          "mismatch in resulting theta value";

  ASSERT_NEAR(Tac_params.x, answer_values.x, 0.01) <<" Multiplication of two transforms using * failed: Value"
                                                          "mismatch in resulting x value";
  ASSERT_NEAR(Tac_params.y, answer_values.y, 0.01) <<" Multiplication of two transforms using * failed: Value"
                                                          "mismatch in resulting y value";
}


TEST(RigidTransform2D, mutiplyAssignTransforms)
{
  rigid2d::Vector2D t_ab = {1.0, 2.0}, t_bc = {4.0, -2.0};
  double theta_ab = 1.0, theta_bc = -2.14;
  rigid2d::Transform2D Tab(t_ab, theta_ab), Tbc(t_bc, theta_bc);
  auto Tac_params = rigid2d::TransformParameters{-1.14, 4.84415119,4.28527933};
  Tab *= Tbc;
  rigid2d::TransformParameters answer_values = Tab.displacement();

  ASSERT_NEAR(Tac_params.theta, answer_values.theta, 0.01) <<" Multiplication of two transforms using *= failed: Value"
                                                          "mismatch in resulting theta value";

  ASSERT_NEAR(Tac_params.x, answer_values.x, 0.01) <<" Multiplication of two transforms using *= failed: Value"
                                                          "mismatch in resulting x value";
  ASSERT_NEAR(Tac_params.y, answer_values.y, 0.01) <<" Multiplication of two transforms using *= failed: Value"
                                                          "mismatch in resulting y value";
}

TEST(RigidTransform2D, transformVector)
{

  rigid2d::Vector2D t_ab = {1.0, 2.0}, pb={10.0, 11.0}, answer{-2.85315777, 16.35803521};
  double theta_ab = 1.0;
  rigid2d::Transform2D Tab(t_ab, theta_ab);
  auto pa = Tab(pb);
  ASSERT_NEAR(answer.x, pa.x, 0.01)<<"Transforming a vector through a transformation matrix failed."
                                   "Mismatch in x value";
  ASSERT_NEAR(answer.y, pa.y, 0.01)<<"Transforming a vector through a transformation matrix failed."
                                   "Mismatch in y value";
}

TEST(RigidTransform2D, transformTwist)
{

  rigid2d::Vector2D t_ab = {3.0, 4.0};
  double theta_ab = 2.0;
  rigid2d::Transform2D Tab(t_ab, theta_ab);
  rigid2d::Twist2D Vb = {7.0, 8.0, 9.0}, answer= {7.0, 16.4871, -17.4709};
  auto Va = Tab(Vb);

  ASSERT_NEAR(Va.wz, answer.wz, 0.01)<<"Transforming a twist through a transformation matrix failed."
                                   "Mismatch in angular velocity z";

  ASSERT_NEAR(Va.vx, answer.vx, 0.01)<<"Transforming a twist through a transformation matrix failed."
                                   "Mismatch in linear velocity x";

  ASSERT_NEAR(Va.vy, answer.vy, 0.01)<<"Transforming a twist through a transformation matrix failed."
                                   "Mismatch in linear velocity y";
}

TEST(RigidTransform2D, Transforminv)
{

  rigid2d::Vector2D t_ab = {-7.6, 4.4};
  double theta_ab = -4.3;
  rigid2d::TransformParameters answer{-1.9831853071795866, -7.0772038, -5.19934476};
  rigid2d::Transform2D Tab(t_ab, theta_ab), Tba;
  Tba = Tab.inv();
  auto transform_params = Tba.displacement();

  ASSERT_NEAR(answer.theta, transform_params.theta, 0.01)<<"Inverting a transform failed"
                                    "Mismatch in final theta value";

  ASSERT_NEAR(answer.x, transform_params.x, 0.01)<<"Inverting the transformation failed"
                                   "Mismatch in final translation x value";

  ASSERT_NEAR(answer.y, transform_params.y, 0.01)<<"Inverting the transform failed"
                                   "Mismatch in final translation y value";
}


TEST(RigidTransform2D, valueGetter)
{

  rigid2d::Vector2D t_ab = {1.0, 2.0};
  double theta_ab = 1.0;
  rigid2d::Transform2D Tab(t_ab, theta_ab);
  auto stored_values = Tab.displacement();
  ASSERT_NEAR(stored_values.theta, 1.0, 0.1)<<"Error in getter function of private variables in transform"
                                              "value mismatch in theta";
  ASSERT_NEAR(stored_values.x, 1.0, 0.1)<<"Error in getter function of private variables in transform"
                                              "value mismatch in x";
  ASSERT_NEAR(stored_values.y, 2.0, 0.1)<<"Error in getter function of private variables in transform"
                                              "value mismatch in y";
}

TEST(RigidTransform2D, inputValuesTransform)
{
  std::stringstream sso;
  rigid2d::Transform2D T1;
  std::string input = "2.0 6.4 -7.5";
  //sso<<2.0<<6.4<<-7.5;
  sso<<input;
  sso>> T1;
  auto T1Params = T1.displacement();
  ASSERT_NEAR(T1Params.theta, 2.0, 0.1)<<"Error in getting transform values from input stream"
                                              "value mismatch in theta";
  ASSERT_NEAR(T1Params.x, 6.4, 0.1)<<"Error in getting transform values from input stream"
                                              "value mismatch in translation x value";

  ASSERT_NEAR(T1Params.y, -7.5, 0)<<"Error in getting transform values from input stream"
                                              "value mismatch in translation y value";
}

TEST(RigidTransform2D, outputValuesTransform)
 {
 
   std::stringstream sso;
   rigid2d::Vector2D t_1 = {1.0, 2.0};
   std::string expected_output = "dtheta:1  dx:1  dy:2", printed_output;
   double theta_1 = 1.0;
   rigid2d::Transform2D T1(t_1, theta_1);
   sso<<T1;
   sso>>printed_output;
   sso>>printed_output;
   sso>>printed_output;
   ASSERT_EQ(expected_output, sso.str())<<"Error in printing transform values to screen";
 }

TEST(RigidTransform2D, inputValuesVector)
{
  std::stringstream sso;
  std::string input = "1.0 2.0";
  rigid2d::Vector2D P1;
  sso<<input;
  sso>>P1;
  ASSERT_NEAR(P1.x, 1.0, 0.1) << "Error in getting inputs for a vector. Mismatch in x values";
  ASSERT_NEAR(P1.y, 2.0, 0.1) << "Error in getting inputs for a vector. Mismatch in y values";
}

TEST(RigidTransform2D, outputValuesVector)
{
 std::string expected_output="1.1  -2\n";
 std::stringstream sso;
 rigid2d::Vector2D T1{1.1, -2.0};
 sso<<T1;
 ASSERT_EQ(sso.str(), expected_output) << "Error in printing vector 2D values";
}

TEST(RigidTransform2D, inputValuesTwist)
{

  std::stringstream sso;
  std::string input = "1 -2.2 3.3";
  rigid2d::Twist2D V1{1.0, -2.2, 3.3};
  sso<<input;
  sso>>V1;
  ASSERT_NEAR(V1.wz, 1.0, 0.1) << "Error in getting inputs for a twist. Mismatch in omega z values";
  ASSERT_NEAR(V1.vx, -2.2, 0.1) << "Error in getting inputs for a twist. Mismatch in linear vx values";
  ASSERT_NEAR(V1.vy, 3.3, 0.1) << "Error in getting inputs for a twist. Mismatch in linear vy values";
}

TEST(RigidTransform2D, outputValuesTwist)
{

 std::string expected_output="1.1  -2  4.3\n";
 std::stringstream sso;
 rigid2d::Twist2D T1{1.1, -2.0, 4.3};
 sso<<T1;
 ASSERT_EQ(sso.str(), expected_output) << "Error in printing 2D twist to screen";

}

TEST(RigidTransform2D, divideVector)
{

 rigid2d::Vector2D T1{1.1, -2.0};
 auto T2 = T1 / 2.0;
 ASSERT_NEAR(T2.x, 0.55, 0.01) <<" Error when dividing a vector by a scalar. Mismatch in x value";
 ASSERT_NEAR(T2.y, -1.0, 0.01) <<" Error when dividing a vector by a scalar. Mismatch in x value";

}

TEST(RigidTransform2D, normaliseVector)
{

 rigid2d::Vector2D T1{1.0, -3.0};
 T1.normalise();
 ASSERT_NEAR(T1.x, 0.31622776601683794, 0.01) <<" Error when dividing a vector by a scalar. Mismatch in x value";
 ASSERT_NEAR(T1.y, -0.9486832980505138 , 0.01) <<" Error when dividing a vector by a scalar. Mismatch in x value";

}

TEST(RigidTransform2D, transformObjectCreation)
{
  rigid2d::Vector2D t{1.2, -3.1};
  double angle = 1.6;
  rigid2d::Transform2D T1, T2(angle), T3(t), T4(t, angle);
  auto T1Params = T1.displacement();
  auto T2Params = T2.displacement();
  auto T3Params = T3.displacement();
  auto T4Params = T4.displacement();

  ASSERT_NEAR(T1Params.x, 0.0, 0.01)<<"Error in creating a no argument transformation object"
                                      "The default transformation is NOT identity";

  ASSERT_NEAR(T1Params.y, 0.0, 0.01)<<"Error in creating a no argument transformation object"
                                      "The default transformation is NOT identity";

  ASSERT_NEAR(T1Params.theta, 0.0, 0.01)<<"Error in creating a no argument transformation object"
                                      "The default transformation is NOT identity";

  ASSERT_NEAR(T2Params.theta, angle, 0.01)<<"Error in creating a pure rotation transformation object"
                                      " Mismatch in angle recorded";

  ASSERT_NEAR(T2Params.x, 0, 0.01)<<"Error in creating a pure rotation transformation object"
                                      " x translation is not zero";

  ASSERT_NEAR(T2Params.y, 0, 0.01)<<"Error in creating a pure rotation transformation object"
                                      " y translation is not zero";

  ASSERT_NEAR(T3Params.theta, 0, 0.01)<<"Error in creating a pure translation transformation object"
                                        "angle is not zero ";

  ASSERT_NEAR(T3Params.x, t.x, 0.01)<<"Error in creating a pure translation transformation object"
                                        " Mismatch in translation x value";
  
  ASSERT_NEAR(T3Params.y, t.y, 0.01)<<"Error in creating a pure translation transformation object"
                                        " Mismatch in translation y value";

  ASSERT_NEAR(T4Params.theta, angle, 0.01)<<"Error in creating a full transformation object"
                                           " Mismatch in theta value recorded"; 

  ASSERT_NEAR(T4Params.x, t.x, 0.01)<<"Error in creating a full transformation object"
                                           " Mismatch in translation x recorded"; 

  ASSERT_NEAR(T4Params.y, t.y, 0.01)<<"Error in creating a full transformation object"
                                           " Mismatch in translation y recorded"; 

  ASSERT_NEAR(T4Params.theta, angle, 0.01)<<"Error in creating a full transformation object"
                                           " Mismatch in theta values recorded"; 
}

TEST(RigidTransform2D, twistIntegration)
{
  rigid2d::Twist2D V1{3.4, -1.2, 4.8};
  auto T1 = rigid2d::integrateTwist(V1);
  auto T1Params = T1.displacement();
  ASSERT_NEAR(T1Params.theta, -2.8831853071795868, 0.01) <<" Error in integrating twist"
                                                           " Mismatch in estimating theta value";

  ASSERT_NEAR(T1Params.x, -2.68646529, 0.01) <<" Error in integrating twist"
                                               " Mismatch in estimating x value";

  ASSERT_NEAR(T1Params.y, -1.05492798, 0.01) <<" Error in integrating twist"
                                               " Mismatch in estimating y value";
}

TEST(RigidTransform2D, vectorAdditionOverloading)
{

  rigid2d::Vector2D V1{1.3, 4.5}, V2{2.4, -6.7}, answer{3.7, -2.2};
  auto V3 = V1 + V2;
  V1 += V2;

  ASSERT_NEAR(V3.x, answer.x, 0.01) <<"Error in adding two vector. Problem with overloading + operator"
                                        "Mismatch in estimating x value";
  ASSERT_NEAR(V3.y, answer.y, 0.01) <<"Error in adding two vector. Problem with overloading + operator" 
                                       "Mismatch in estimating y value";
  ASSERT_NEAR(V1.x, answer.x, 0.01) <<"Error in adding two vector. Problem with overloading += operator"
                                       "Mismatch in estimating x value";
  ASSERT_NEAR(V1.y, answer.y, 0.01) <<"Error in adding two vector. Problem with overloading += operator" 
                                      "Mismatch in estimating y value";

}


TEST(RigidTransform2D, vectorSubtractionOverloading)
{

  rigid2d::Vector2D V1{1.3, 4.5}, V2{2.4, -6.7}, answer{-1.1, 11.2};
  auto V3 = V1 - V2;
  V1 -= V2;

  ASSERT_NEAR(V3.x, answer.x, 0.01) <<"Error in adding two vector. Problem with overloading + operator"
                                       "Mismatch in estimating x value";
  ASSERT_NEAR(V3.y, answer.y, 0.01) <<"Error in adding two vector. Problem with overloading + operator" 
                                      "Mismatch in estimating y value";
  ASSERT_NEAR(V1.x, answer.x, 0.01) <<"Error in adding two vector. Problem with overloading += operator"
                                       "Mismatch in estimating x value";
  ASSERT_NEAR(V1.y, answer.y, 0.01) <<"Error in adding two vector. Problem with overloading += operator" 
                                      "Mismatch in estimating y value";

}


TEST(RigidTransform2D, vectorMultiplicationOverloading)
{

  double scaling_factor=2.3;
  rigid2d::Vector2D V1{1.3, 4.5}, answer{2.989999, 10.35};
  auto V2 =scaling_factor * V1 ;
  auto V3 = V1 * scaling_factor;
  V1 *= scaling_factor;

  ASSERT_NEAR(V2.x, answer.x, 0.01) <<"Error in vector multiplication. Problem with overloading multiplication (float * vector) operator"
                                       "Mismatch in estimating x value";
  ASSERT_NEAR(V2.y, answer.y, 0.01) <<"Error in vector multiplication. Problem with overloading multiplication (float * vector) operator" 
                                      "Mismatch in estimating y value";
  ASSERT_NEAR(V3.x, answer.x, 0.01) <<"Error in vector multiplication. Problem with overloading multiplication (vector * float) operator"
                                       "Mismatch in estimating x value";
  ASSERT_NEAR(V3.y, answer.y, 0.01) <<"Error in vector multiplication. Problem with overloading multiplication (vector * float) operator" 
                                      "Mismatch in estimating y value";
  ASSERT_NEAR(V1.x, answer.x, 0.01) <<"Error in vector multiplication. Problem with overloading multiplication *= operator"
                                       "Mismatch in estimating x value";
  ASSERT_NEAR(V1.y, answer.y, 0.01) <<"Error in vector multiplication. Problem with overloading multiplication *= operator" 
                                      "Mismatch in estimating y value";
}

TEST(RigidTransform2D, vectorLength)
{
  rigid2d::Vector2D V1{-1.4, 7.8};
  double answer = 7.924645;
  double vec_length = rigid2d::length(V1);

  ASSERT_NEAR(vec_length, answer, 0.01) <<"Error in calculating vector length";

}

TEST(RigidTransform2D, vectorDistance)
{
  rigid2d::Vector2D V1{1,2}, V2{3, 4};
  double answer = 2.8284271;
  double vec_distance = rigid2d::distance(V1, V2);

  ASSERT_NEAR(vec_distance, answer, 0.01) <<"Error in calculating distance between two vectors";
}


TEST(RigidTransform2D, vectorAngle)
{
  rigid2d::Vector2D V1{1.5, -0.24};
  double answer = -0.15865526;
  double vec_angle = rigid2d::angle(V1);

  ASSERT_NEAR(vec_angle, answer, 0.01) <<"Error in calculating the angle of a vector";
}

TEST(DiffDrive, twistToWheelVelocitiesConversion)
{

 rigid2d::Twist2D Twist2beApplied = {0.0, 0.3, 0.0};
 double wheel_radius = 3.0, wheel_base=6.0;
 rigid2d::Transform2D initialPose;
 rigid2d::DiffDrive diffCar(initialPose, wheel_base, wheel_radius);
 auto WheelVelocities = diffCar.twistToWheelVelocities(Twist2beApplied);
 ASSERT_NEAR(WheelVelocities.left, WheelVelocities.right, 0.01) <<"Error in calculating wheel velocities from given twists";

 Twist2beApplied = {2.8, 0.0, 0.0};
 WheelVelocities = diffCar.twistToWheelVelocities(Twist2beApplied);
 ASSERT_NEAR(WheelVelocities.left, -WheelVelocities.right, 0.01) <<"Error in calculating wheel velocities from given twists";

 Twist2beApplied = {1.0, -1.0, 0.0};
 double left_wheel_answer=(-wheel_base/2-1) / wheel_radius;
 double right_wheel_answer=(wheel_base/2-1) / wheel_radius;
 WheelVelocities = diffCar.twistToWheelVelocities(Twist2beApplied);
 ASSERT_NEAR(WheelVelocities.right, right_wheel_answer, 0.01) <<"Error in calculating wheel velocities from given twists";
 ASSERT_NEAR(WheelVelocities.left, left_wheel_answer, 0.01) <<"Error in calculating wheel velocities from given twists";
}


TEST(DiffDrive, wheelVelocitiesToTwistConversion)
{

 double wheel_radius = 3.0, wheel_base=6.0;
 rigid2d::Transform2D initialPose;
 rigid2d::DiffDrive diffCar(initialPose, wheel_base, wheel_radius);
 rigid2d::WheelVelocities velocities = {1.0, 1.0};
 auto BodyTwist = diffCar.WheelVelocitiestoTwist(velocities);
 ASSERT_NEAR(BodyTwist.wz, 0.00, 0.01) <<"Error in converting twist to wheel velocities. The twist has non-zero wz component"
                                         "for equal wheel velocities";
 ASSERT_NEAR(BodyTwist.vy, 0.00, 0.01) <<"Error in converting twist to wheel velocities. The twist has non-zero vy component"
                                         "for equal wheel velocities";

 velocities = {-1.0, 1.0};
 BodyTwist = diffCar.WheelVelocitiestoTwist(velocities);
 ASSERT_NEAR(BodyTwist.vx, 0.00, 0.01) <<"Error in converting twist to wheel velocities. The twist has non-zero vx component"
                                         "for equal and opposite wheel velocities";
 ASSERT_NEAR(BodyTwist.vy, 0.00, 0.01) <<"Error in converting twist to wheel velocities. The twist has non-zero vy component"
                                         "for equal and opposite wheel velocities";

 velocities = {-0.3, 1.0};
 double wz_answer = - wheel_radius / wheel_base * velocities.left +
                    wheel_radius / wheel_base * velocities.right;
 double vx_answer = wheel_radius / 2 * velocities.left + wheel_radius / 2 * velocities.right;
 BodyTwist = diffCar.WheelVelocitiestoTwist(velocities);
 ASSERT_NEAR(BodyTwist.vy, 0.00, 0.01) <<"Error in converting twist to wheel velocities. The body twist has non-zero vy component";
 ASSERT_NEAR(BodyTwist.wz, wz_answer, 0.01) <<"Error in converting twist to wheel velocities. The twist has wrong wz component";
 ASSERT_NEAR(BodyTwist.vx, vx_answer, 0.01) <<"Error in converting twist to wheel velocities. The twist has wrong vx component";

}

TEST(DiffDrive, UpdataOdometry)
{

 double wheel_radius = 3.0, wheel_base=6.0;
 rigid2d::Transform2D initialPose;
 rigid2d::DiffDrive diffCar(initialPose, wheel_base, wheel_radius);

 double phi_left=1.0, phi_right=1.0;
 diffCar.UpdateOdometry(phi_left, phi_right);
 auto currentPose = diffCar.returnPose();
 ASSERT_NEAR(currentPose.x, wheel_radius * phi_left, 0.01) <<"Error in updating odometry for moving in the body vx direction";
 ASSERT_NEAR(currentPose.y, 0 ,0.01) <<"Error in updating odometry for moving in the body vx direction";
 ASSERT_NEAR(currentPose.theta, 0 ,0.01) <<"Error in updating odometry for moving in the body vx direction";
 phi_left=1.0, phi_right=-1.0;
 diffCar.reset(initialPose);
 diffCar.UpdateOdometry(phi_left, phi_right);
 currentPose = diffCar.returnPose();
 ASSERT_NEAR(currentPose.theta, -1.0, 0.01) <<"Error in updating odometry for rotating in place";
 ASSERT_NEAR(currentPose.x, 0.00, 0.01) <<"Error in updating odometry for rotating in place";
 ASSERT_NEAR(currentPose.y, 0.00, 0.01) <<"Error in updating odometry for rotating in place";


 phi_left=1.0, phi_right=0.5;
 diffCar.reset(initialPose);
 diffCar.UpdateOdometry(phi_left, phi_right);
 currentPose = diffCar.returnPose();
 rigid2d::Twist2D equivalentTwist = {-wheel_radius / 2 / wheel_base, 3 * wheel_radius / 4, 0.0};
 auto finalTransform = rigid2d::integrateTwist(equivalentTwist);
 auto answer = finalTransform.displacement();
 ASSERT_NEAR(answer.x, currentPose.x, 0.01) <<"Error in updating odometry for a rotate + translate case. Mismatch in x value";
 ASSERT_NEAR(answer.y, currentPose.y, 0.01) <<"Error in updating odometry for a rotate + translate case. Mismatch in y value";
 ASSERT_NEAR(answer.theta, currentPose.theta, 0.01) <<"Error in updating odometry for a rotate + translate case. Mismatch in theta value";

 phi_left=0.0, phi_right=0.0;
 diffCar.reset(initialPose);
 diffCar.UpdateOdometry(phi_left, phi_right);
 currentPose = diffCar.returnPose();
 ASSERT_NEAR(currentPose.x, 0, 0.01) <<"Error in updating odometry for a base case of zero velocity";
 ASSERT_NEAR(currentPose.y, 0 ,0.01) <<"Error in updating odometry for a base case of zero velocity";
 ASSERT_NEAR(currentPose.theta, 0 ,0.01) <<"Error in updating odometry for a base case of zero velocity";


 phi_left=1.0, phi_right=1.0;
 diffCar.UpdateOdometry(phi_left, phi_right);
 diffCar.UpdateOdometry(phi_left, phi_right);
 diffCar.UpdateOdometry(phi_left, phi_right);
 currentPose = diffCar.returnPose();
 ASSERT_NEAR(currentPose.x, wheel_radius * phi_left * 3, 0.01) <<"Error in updating odometry for a series of moition commands";
 ASSERT_NEAR(currentPose.y, 0 ,0.01) <<"Error in updating odometry for a series of moition commands";
 ASSERT_NEAR(currentPose.theta, 0 ,0.01) <<"Error in updating odometry for a series of moition commands";

 phi_left=1.0, phi_right=-1.0;
 diffCar.reset(initialPose);
 diffCar.UpdateOdometry(phi_left, phi_right);
 diffCar.UpdateOdometry(phi_left, phi_right);
 diffCar.UpdateOdometry(phi_left, phi_right);
 currentPose = diffCar.returnPose();
 ASSERT_NEAR(currentPose.theta, -3.0, 0.01) <<"Error in updating odometry for a sequence of rotations";
 ASSERT_NEAR(currentPose.x, 0.00, 0.01) <<"Error in updating odometry for a sequence of rotations";
 ASSERT_NEAR(currentPose.y, 0.00, 0.01) <<"Error in updating odometry for a sequence of rotations";


 phi_left=0.00339831, phi_right=0.00339831;
 diffCar.reset(initialPose);
 diffCar.UpdateOdometry(phi_left, phi_right);
 currentPose = diffCar.returnPose();
 ASSERT_NEAR(currentPose.theta, 0.00, 0.01)<<"Error  in updating odometry for a sequence of translation";
 ASSERT_NEAR(currentPose.x, wheel_radius * 0.00339831, 0.01) <<"Error in updating odometry for a sequence of translation";
 ASSERT_NEAR(currentPose.y, 0.00, 0.01) <<"Error in updating odometry for a sequence of translation";


}

TEST(DiffDrive, feedforward)
{

 double wheel_radius = 3.0, wheel_base=6.0;
 rigid2d::Transform2D initialPose(0);
 rigid2d::DiffDrive diffCar(initialPose, wheel_base, wheel_radius);

 rigid2d::Twist2D AppliedTwist = {1.0, 0, 0};
 diffCar.feedforward(AppliedTwist);
 auto currentPose = diffCar.returnPose();

 ASSERT_NEAR(currentPose.x, 0, 0.01) <<"Error in updating odometry using feedforward twist. X translation is non-zero for pure rotation";
 ASSERT_NEAR(currentPose.y, 0 ,0.01) <<"Error in updating odometry using feedforward twist. Y translation is non-zero for pure rotation";
 ASSERT_NEAR(currentPose.theta, 1.0 ,0.01) <<"Error in updating odometry using feedforward twist.";

 AppliedTwist = {0.0, 1.0, 0};
 diffCar.reset(initialPose);
 diffCar.feedforward(AppliedTwist);
 currentPose = diffCar.returnPose();

 ASSERT_NEAR(currentPose.x, 1.0, 0.01) <<"Error in updating odometry using feedforward twist";
 ASSERT_NEAR(currentPose.y, 0 ,0.01) <<"Error in updating odometry using feedforward twist."
                                         "Y translation is non-zero for pure translation along x axis";
 ASSERT_NEAR(currentPose.theta, 0.0 ,0.01) <<"Error in updating odometry using feedforward twist."
                                             "orientation is non-zero for a pure translation along x axis";


 AppliedTwist = {1.0, 1.0, 0};
 diffCar.reset(initialPose);
 diffCar.feedforward(AppliedTwist);
 currentPose = diffCar.returnPose();

 ASSERT_NEAR(currentPose.x, cos(1.5 * rigid2d::PI + 1.0), 0.01) <<"Error in updating odometry using feedforward twist for rotation + translation";
 ASSERT_NEAR(currentPose.y, sin(1.5 * rigid2d::PI + 1.0) + 1.0 ,0.01) <<"Error in updating odometry using feedforward twist for rotation + translation";
 ASSERT_NEAR(currentPose.theta, 1.0 ,0.01) <<"Error in updating odometry using feedforward twist for rotation + translation";



}

TEST(Rigid2d, circularlinkedlist)
{
  rigid2d::Vector2D P1 = {1.0, 2.0}, P2 = {-11.0, -2.0};
  std::vector<rigid2d::Vector2D> traj_points{P1, P2};
  rigid2d::Waypoint waypoints(traj_points);
  // waypoints.trajectoryPoints.addElements(traj_points);
  auto X1 = waypoints.trajectoryPoints.returnNextElement();
  auto X2 = waypoints.trajectoryPoints.returnNextElement();
  auto X3 = waypoints.trajectoryPoints.returnNextElement();

  ASSERT_NEAR(X1.x, P2.x, 0.01) <<"Error in circular linked list. First point does not match";
  ASSERT_NEAR(X1.y, P2.y, 0.01) <<"Error in circular linked list. First point does not match";

  ASSERT_NEAR(X2.x, P1.x, 0.01) <<"Error in circular linked list. Second point does not match";
  ASSERT_NEAR(X2.y, P1.y, 0.01) <<"Error in circular linked list. Second point does not match";

  ASSERT_NEAR(X3.x, P2.x, 0.01) <<"Error in circular linked list.The list does not cycle back to the first element after the last";
  ASSERT_NEAR(X3.y, P2.y, 0.01) <<"Error in circular linked list. The list does not cycle back to the first element after the last";

}













