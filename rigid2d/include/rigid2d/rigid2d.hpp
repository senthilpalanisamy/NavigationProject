#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include<stdlib.h>
#include<iostream>
#include<math.h>




namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
       return(abs(d1-d2) < epsilon? true: false);
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
      return(deg * rigid2d::PI / 180.0);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
      return(rad * 180 / rigid2d::PI);
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");


    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
      double x, y;

      /// \brief Default constructor for Vector2D
      Vector2D(): x(0), y(0) {}
      /// \brief Parameterised constructor for the class
      /// \param x1 - x co-ordinate value
      /// \param y1 - y co-ordinate value
      constexpr Vector2D(const double x1, const double y1): x(x1), y(y1) {}
      /// \brief Overloading division operator. Division is only defined for division
      /// by a scalar
      /// \param divisor - the scalar number to divide the vector by
      /// \return The resultant vector obtained by dividing the vector element-wise
      /// by the scalar
      Vector2D operator/=(const double divisor);
      /// \brief Overloading +- operator for vector addition
      /// \param rhsVector - the vector which should be added to the vector
      /// on the right hand side of the equation
      /// \return A vector whose sum is equal to the sum of two vectors
      Vector2D operator+=(const Vector2D & rhsVector);
      /// \brief Overloading of -= operator for vector subtraction
      /// \param rhsVector - the vector which should be subtracted from the
      /// vector on the right hand side of the equation.
      /// \return A vector whose value is equal to the element wise subtraction
      /// of the vector on the right from the  vector on the left
      Vector2D operator-=(const Vector2D & rhsVector);
      /// \brief Overloading of *= operator for vector multiplication. It should be
      /// not here that the multiplication is a multiplication by a scalar number
      /// \param scaling_constant - the scalar number which should be multiplied to
      /// every component of the vector
      /// \return A vector whose each element is scaled by the scaling constant 
      /// given
      Vector2D operator*=(const double scaling_constant);
      /// \brief Normalise and return a vector which has unit magnitude and the same
      /// direction as that of the original vector
      void normalise();

    };

    /// \brief vector addition overloading. Two vectors could be added element wise
    /// using + operator
    /// \param lhsVector - The vector element that lies to the left of + operator
    /// \param rhsVector - The vector element that lies to the right of + operator
    /// \return A vector which is equal to the sum of the two vectors
    Vector2D operator+(Vector2D  lhsVector, const Vector2D & rhsVector);
    /// \brief vector subtraction overloading. Two vectors could be subtracted element wise
    /// using - operator
    /// \param lhsVector - The vector element that lies to the left of - operator or the 
    /// vector from which the other vector should be subtracted
    /// \param rhsVector - The vector element that lies to the right of - operator or the
    /// vector that should be subtracted from the other vector
    /// \return A vector whose value is equal to the subtraction of the vector
    /// on the right from the vector on the left
    Vector2D operator-(Vector2D  lhsVector, const Vector2D & rhsVector);
    /// \brief vector multiplication overloading using * operator. A vector can 
    /// be scaled element wise by a scalar value
    /// \param lhsVector - The vector element that should be scaled. It can lie
    /// on either side of the * operator provided the other number is scalar.
    /// \param scaling_constant - The scalar number that should be multiplied 
    /// to every element of the vector. This can lie on either side of the star 
    /// operator
    /// \return A vector whose every element is scaled by the scaling constant
    /// given
    Vector2D operator*(Vector2D  lhsVector, const double scaling_constant);
    /// \brief vector multiplication overloading using * operator. A vector can 
    /// be scaled element wise by a scalar value
    /// \param rhsVector - The vector element that should be scaled. It can lie
    /// on either side of the * operator provided the other number is scalar.
    /// \param scaling_constant - The scalar number that should be multiplied 
    /// to every element of the vector. This can lie on either side of the star 
    /// operator
    /// \return A vector whose every element is scaled by the scaling constant
    /// given
    Vector2D operator*(const double scaling_constant, Vector2D rhsVector);
    /// \brief overloading of division operator to implement the element-wise division
    /// of a vector by a scalar value.  It should be noted that the scalar value
    /// has to lie on the right side of division operator. Vice-versa is not
    /// permitted.
    /// \param v1 - The vector to be divided
    /// \param divisor - The scalar number through which each element of the
    /// vector should be divided by
    /// \return A vector whose every element is divided by the value given 
    /// as divisor.
    Vector2D operator/(Vector2D v1, const double divisor);
    /// \brief Returns the length L2 norm of a given vector
    /// \param V - Vector whose length is to be found
    /// \return length of the vecor
    constexpr double length(Vector2D V)
    {
     return (pow(pow(V.x, 2)+pow(V.y, 2), 0.5));
    }
    /// \brief Returns the distance between two vectors
    /// \param v1 - vector 1 for finding distance
    /// \param v2 - vector 2 for finding distance
    /// \return distance between the vectors
    constexpr double distance(Vector2D v1, Vector2D v2)
    {
      return(pow(pow(v1.x-v2.x, 2) + pow(v1.y - v2.y, 2), 0.5));

    }
    /// \brief Find the angle of the vector with respect to standard cartesian axis
    /// The angle returned is between [-pi,pi)
    /// \param v1 - vector whose angle is to be found
    /// \return angle of the vector 
    constexpr double angle(Vector2D v1)
    {
      return (atan2(v1.y, v1.x));
    }
    /// \brief Normalizes the given angle and returns an angle that is between [-pi,pi)
    /// \param rad - Angle in radians which is to be normalised
    /// \return normalized angled
    constexpr double normalize_angle(double rad)
    {
      return(atan2(sin(rad), cos(rad)));
    }
    /// \brief A static assertion check for checking the normalize functionality
    static_assert(almost_equal(normalize_angle(5.28), -1.0031, 1.0e-2), "is_zero failed");



    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);


    /// \brief A structure that represents a twist in a axis angle format. The axis
    /// is normalized as per screw axis normalizing conventions and the 
    /// angle is extracted out such that angle * normalised_scre_axis = Twist
    struct AxisAngle
    {
      double vx, vy, wz, angle;
    };

    /// \brief A 2-Dimensional Twist vector
    struct Twist2D
    {
       double wz=0.0, vx=0.0, vy=0.0;
       AxisAngle return_axis_angle_representation() const;
    };

    /// \brief A structure that is used in the getter function of Transform2D class
    /// to allow a view of its private variables.
    struct TransformParameters
    {
      double theta, x, y;
    };

    /// \brief output a 2 dimensional twist vector as [omegaz linearx lineary]
    /// os - stream to output to
    /// v - twist vector to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & v);

    /// \brief input a 2 dimensional twist vector
    /// is - stream from which to read
    /// v [out] - output 2 dimensional twist vector [omegaz linearx lineary]
    std::istream & operator>>(std::istream & is, Twist2D & v);


    /// \brief A 2-Dimensional Pose
    struct Pose2D
    {
       double theta=0.0, x=0.0, y=0.0;
    };





    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();


        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;
        /// \brief Adjoint transformation of a twist from one frame to another
        /// \param v - the twist which is to be transformed from one frame
        /// to another as per the transformation matrix specified
        /// \return The twist transformed to another frame
        Twist2D operator()(Twist2D v) const;
        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);
        // operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
        /// \brief A getter function for viewing the private variables of the
        /// transform class
        /// \return Returns a structure containing the translation and rotation
        /// parameters
        TransformParameters displacement() const;


    private:
        /// directly initialize, useful for forming the inverse
        Transform2D(double theta, double ctheta, double stheta, double x, double y);
        double theta, ctheta, stheta, x, y; // angle, sin, cos, x, and y

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function can be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief Integrate a given twist to a transformation matrix
    /// \param V1 - The twist representing the eucleadian motion to be 
    /// integrated
    /// \param totalTime - The time period over which the twist should 
    /// be integrated. By default the time period is assumed to be 1 units.
    /// \return - A Transform2D object which represents the movement of the 
    /// frame
    Transform2D integrateTwist(const Twist2D& V1, double totalTime=1.0);

    /// \brief A structure for velocities of the wheels of a differential drive robot
    struct WheelVelocities
    {
    double left, right;
    };

    /// \brief A class for implementing all functionalities of differential
    /// drive robot. It includes functions used for controls like switching
    /// from body twist to wheel velocities and vice-versa. The class also
    /// holds two mechanisms for odometry by computing the motion due to a 
    /// feed forward control or by updating encoder readings.
    class DiffDrive
    {
      public:
      /// \brief the default constructor creates a robot at (0,0,0), with a 
      /// fixed wheel base and wheel radius
      DiffDrive();
      /// \brief create a DiffDrive model by specifying the pose, and geometry
      ///
      /// \param pose - the current position of the robot
      /// \param wheel_base - the distance between the wheel centers
      /// \param wheel_radius - the raidus of the wheels
      DiffDrive(Transform2D robotPose, double wheel_base, double wheel_radius);

      /// \brief determine the wheel velocities required to make the robot
      /// move with the desired linear and angular velocities
      /// \param twist - the desired twist in the body frame of the robot
      /// \returns - the wheel velocities to use
      WheelVelocities twistToWheelVelocities(Twist2D BodyTwist, double totalTime=1.0);

      /// \brief determine the body twist of the robot from its wheel velocities
      /// \param vel - the velocities of the wheels, assumed to be held constant
      /// for one time unit
      /// \returns twist in the original body frame of the
      Twist2D WheelVelocitiestoTwist(WheelVelocities velocities);
      /// \brief Update the robot's odometry based on the current encoder readings
      /// \param left - the left encoder angle (in radians)
      /// \param right - the right encoder angle (in radians)
      void UpdateOdometry(double phiLeft, double phiRight);
      /// \brief update the odometry of the diff drive robot, assuming that
      /// it follows the given body twist for one time  unit
      /// \param cmd - the twist command to send to the robot
      void feedforward(Twist2D BodyTwist, double totalTime=1.0);
      // void feedforward(Twist2D BodyTwist);
      /// \brief get the current pose of the robot
      /// \return A view of all private datas representing the pose of the robot
      TransformParameters returnPose();
      /// \brief get the wheel speeds, based on the last encoder update
      /// \returns the velocity of the wheels, which is equivalent to
      ///  displacement because \Delta T = 1
      WheelVelocities returnLastEncoderVelocities() const;
      /// \brief reset the robot to the given position/orientation
      void reset(Transform2D newPose);
      private:
      /// Private data of the robot include the pose of the robot, its geometry
      /// details like wheelBase and wheelRadius. It also holds the previous
      /// wheel velocities for computing odometry due to a feedforward control
      Transform2D currentPose;
      double wheelBase, wheelRadius;
      WheelVelocities previousWheelVelocities;
    };
}

#endif
