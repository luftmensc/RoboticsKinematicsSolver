#include "../include/KinematicsLib/ThreeDOFRRR.hpp"
#include <cmath>
#include <iostream>
#include <limits>

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

Eigen::Vector3d ThreeDOFRobot::getJointAngles(void)
{
    return jointAngles;
}

Eigen::Vector3d ThreeDOFRobot::getLinkLengths(void)
{
    return linkLengths;
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

bool ThreeDOFRobot::isEndEffectorInCircle(const double &circle_x, const double &circle_y, const double &r, const double &th1, const double &th2, const double &th3)
{
    // Set the joint angles
    setJointAngRadians({th1, th2, th3});

    // Solve the forward kinematics
    Eigen::Vector3d position_XYW = solveForwardKinematicsDH();
    std::cout << "End effector position (DH): " << position_XYW.transpose() << std::endl;
    // Check if the end effector is within the circle
    return (position_XYW[0] - circle_x) * (position_XYW[0] - circle_x) + (position_XYW[1] - circle_y) * (position_XYW[1] - circle_y) <= r * r;
    
}

Eigen::Vector3d ThreeDOFRobot::solveInverseKinematics(const Eigen::Vector3d &endEffectorXYW)
{
    double distanceToTarget = sqrt(endEffectorXYW[0] * endEffectorXYW[0] + endEffectorXYW[1] * endEffectorXYW[1]);
    
    // Check if the target is within reach
    double maxReach = linkLengths[0] + linkLengths[1] + linkLengths[2];

    if(endEffectorXYW[0]*endEffectorXYW[0] + endEffectorXYW[1]*endEffectorXYW[1] > maxReach*maxReach)
    {
        std::cerr << "Warning: Target is out of reach." << std::endl;
        return Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN());
    }
    // Convert joint angles from degrees to radians
    double phi = endEffectorXYW[2];
    double p_2x = endEffectorXYW[0] - linkLengths[2] * cos(phi);
    double p_2y = endEffectorXYW[1] - linkLengths[2] * sin(phi);

    double th2 = acos((p_2x * p_2x + p_2y * p_2y - linkLengths[0] * linkLengths[0] - linkLengths[1] * linkLengths[1]) / (2 * linkLengths[0] * linkLengths[1]));
    double th1 = atan2(p_2y, p_2x) - atan2(linkLengths[1] * sin(th2), linkLengths[0] + linkLengths[1] * cos(th2));

    double th3 = phi - th1 - th2;

    return Eigen::Vector3d(th1, th2, th3);


}