#include "../include/KinematicsLib/ThreeDOFRRR.hpp"
#include <cmath>
#include <iostream>

/*
 * This is the implementation file for the ThreeDOFRRR class.
    * The class is used to model a 3-DOF RRR robot and solve the forward kinematics problems.
    * For the Forward Kinematics problem, the class provides two methods:
    * 1. solveForwardKinematicsDH: This method solves the forward kinematics problem using the Denavit-Hartenberg (DH) parameters.
    * 2. solveForwardKinematicsLS: This method solves the forward kinematics problem using the brute force algebraic model.
 */

ThreeDOFRobot::ThreeDOFRobot() {
    jointAngles = Eigen::Vector3d::Zero();
    linkLengths = Eigen::Vector3d::Zero();
    calculatedTransformationMatrix = Eigen::Matrix4d::Identity();
}

ThreeDOFRobot::ThreeDOFRobot(const std::vector<double> &jointAngles, const std::vector<double> &linkLengths)
{
    setJointAngRadians(Eigen::Vector3d(jointAngles[0], jointAngles[1], jointAngles[2]));
    setLinkLengths(Eigen::Vector3d(linkLengths[0], linkLengths[1], linkLengths[2]));
}

ThreeDOFRobot::~ThreeDOFRobot() {}

void ThreeDOFRobot::setJointAngRadians(const Eigen::Vector3d &angles)
{
    jointAngles = angles;
}

void ThreeDOFRobot::setLinkLengths(const Eigen::Vector3d &lengths)
{
    linkLengths = lengths;
}

Eigen::Vector3d ThreeDOFRobot::solveForwardKinematicsDH() //Modeling with Denevit-Hartenberg (DH) Parameters, i'm using mainly this one
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    double alpha = 0.0;
    double d = 0.0;
    for (size_t i = 0; i < jointAngles.size(); ++i)
    {
        T *= calculateTransformationMatrix(jointAngles[i], linkLengths[i], alpha, d);
    }
    calculatedTransformationMatrix = T;
    Eigen::Vector3d endEffectorPosition = T.block<3, 1>(0, 3);

    Eigen::Matrix3d rotationMatrix = T.block<3, 3>(0, 0);
    double yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

    return Eigen::Vector3d(endEffectorPosition[0], endEffectorPosition[1], yaw);
}

Eigen::Matrix4d ThreeDOFRobot::calculateTransformationMatrix(const double &theta, const double &a, const double &alpha, const double &d)
{
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;
    return T;
}

Eigen::Vector3d ThreeDOFRobot::solveForwardKinematicsLS() //Brute Force Algebraic Robot Model, i called it LS
{

    // Convert joint angles from degrees to radians
    double theta1_rad = jointAngles[0];
    double theta2_rad = jointAngles[1]; 
    double theta3_rad = jointAngles[2]; 

    double x = 0.0;
    double y = 0.0;
    // Apply the transformations for each joint
    // For joint 1
    x += linkLengths[0] * cos(theta1_rad); // Access using []
    y += linkLengths[0] * sin(theta1_rad); // Access using []

    // For joint 2
    x += linkLengths[1] * cos(theta1_rad + theta2_rad); // Access using []
    y += linkLengths[1] * sin(theta1_rad + theta2_rad); // Access using []

    // For joint 3
    x += linkLengths[2] * cos(theta1_rad + theta2_rad + theta3_rad); // Access using []
    y += linkLengths[2] * sin(theta1_rad + theta2_rad + theta3_rad); // Access using []

    // The orientation of the end effector is the sum of all joint angles
    double theta_p = theta1_rad + theta2_rad + theta3_rad;
    // make it between -pi and pi
    theta_p = std::fmod(theta_p + M_PI, 2 * M_PI) - M_PI;

    // Return the position and orientation of the end effector as an Eigen::Vector3d
    return Eigen::Vector3d(x, y, theta_p);
}
