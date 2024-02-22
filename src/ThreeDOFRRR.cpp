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

ThreeDOFRobot::ThreeDOFRobot(Eigen::Vector3d jointAngles, Eigen::Vector3d linkLengths) : jointAngles(jointAngles), linkLengths(linkLengths) {}

ThreeDOFRobot::ThreeDOFRobot() = default;

ThreeDOFRobot::~ThreeDOFRobot() = default;

void ThreeDOFRobot::setJointAngRadians(const Eigen::Vector3d &angles)
{
    jointAngles = angles;
}

void ThreeDOFRobot::setLinkLengths(const Eigen::Vector3d &lengths)
{
    linkLengths = lengths;
}

Eigen::Vector3d ThreeDOFRobot::getJointAngles(void) const
{
    return jointAngles;
}

Eigen::Vector3d ThreeDOFRobot::getLinkLengths(void) const
{
    return linkLengths;
}

void ThreeDOFRobot::setEndEffectorPosition(const Eigen::Vector3d &position)
{
    endEffectorPosition = position;
}

Eigen::Vector3d ThreeDOFRobot::getEndEffectorPosition(void) const
{
    return endEffectorPosition;
}
Eigen::Vector3d ThreeDOFRobot::solveForwardKinematicsDH() // Modeling with Denevit-Hartenberg (DH) Parameters, i'm using mainly this one
{
    Eigen::Matrix4d T{Eigen::Matrix4d::Identity()};

    // In 3DoF RRR Planar robot, alpha and d are always zero
    double alpha{0.0};
    double d{0.0};

    for (size_t i = 0; i < jointAngles.size(); ++i)
    {
        T *= calculateTransformationMatrix(jointAngles[i], linkLengths[i], alpha, d);
    }

    calculatedTransformationMatrix = T; // Save the calculated transformation matrix

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

Eigen::Vector3d ThreeDOFRobot::solveForwardKinematicsLS() // Brute Force Algebraic Robot Model, i called it LS
{

    // Convert joint angles from degrees to radians
    const double theta1_rad = jointAngles[0];
    const double theta2_rad = jointAngles[1];
    const double theta3_rad = jointAngles[2];

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
    Eigen::Vector3d position_XYW = solveForwardKinematicsDH(); // Modeling with Denevit-Hartenberg (DH) Parameters, soleForwardKinematicsLs can be used also!
    // Check if the end effector is within the circle and return the result
    setEndEffectorPosition(position_XYW);
    return (position_XYW[0] - circle_x) * (position_XYW[0] - circle_x) + (position_XYW[1] - circle_y) * (position_XYW[1] - circle_y) <= r * r;
}

Eigen::Vector3d ThreeDOFRobot::solveInverseKinematics(const Eigen::Vector3d &endEffectorXYW)
{

    // Check if the target is within reach
    const double maxReach = linkLengths[0] + linkLengths[1] + linkLengths[2];

    if (endEffectorXYW[0] * endEffectorXYW[0] + endEffectorXYW[1] * endEffectorXYW[1] > maxReach * maxReach) {
        std::cerr << "Warning: Target is out of reach." << std::endl;
        // Use infinity to indicate out of reach
        return Eigen::Vector3d(std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity());
    }

    const double phi = endEffectorXYW[2];
    double p_2x = endEffectorXYW[0] - linkLengths[2] * cos(phi);
    double p_2y = endEffectorXYW[1] - linkLengths[2] * sin(phi);

    double th2 = acos((p_2x * p_2x + p_2y * p_2y - linkLengths[0] * linkLengths[0] - linkLengths[1] * linkLengths[1]) / (2 * linkLengths[0] * linkLengths[1]));

    double th1 = atan2(p_2y, p_2x) - atan2(linkLengths[1] * sin(th2), linkLengths[0] + linkLengths[1] * cos(th2));

    // Normalize the angles between -pi and pi
    auto normalizeRadians = [](double angle)
    {
        angle = std::fmod(angle, 2 * M_PI);
        if (angle > M_PI)
            angle -= 2 * M_PI;
        else if (angle <= -M_PI)
            angle += 2 * M_PI;
        return angle;
    };

    double th3 = phi - th1 - th2;
    th3 = normalizeRadians(th3);
    return Eigen::Vector3d(th1, th2, th3);
}

std::vector<Eigen::Vector3d> ThreeDOFRobot::solveInverseKinematics2Solution(const Eigen::Vector3d &endEffectorXYW)
{
    std::vector<Eigen::Vector3d> solutions;

    // Check if the target is within reach
    const double maxReach = linkLengths[0] + linkLengths[1] + linkLengths[2];

    if (endEffectorXYW[0] * endEffectorXYW[0] + endEffectorXYW[1] * endEffectorXYW[1] > maxReach * maxReach) {
        std::cerr << "Warning: Target is out of reach." << std::endl;

        // Use infinity to indicate out of reach
        Eigen::Vector3d solution{std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity()};
        solutions.push_back(solution);
        solutions.push_back(solution);
        return solutions;
        
    }

    double phi = endEffectorXYW[2];
    double p_2x = endEffectorXYW[0] - linkLengths[2] * cos(phi);
    double p_2y = endEffectorXYW[1] - linkLengths[2] * sin(phi);

    double th2 = acos((p_2x * p_2x + p_2y * p_2y - linkLengths[0] * linkLengths[0] - linkLengths[1] * linkLengths[1]) / (2 * linkLengths[0] * linkLengths[1]));

    double th1 = atan2(p_2y, p_2x) - atan2(linkLengths[1] * sin(th2), linkLengths[0] + linkLengths[1] * cos(th2));

    // Normalize the angles between -pi and pi
    auto normalizeRadians = [](double angle)
    {
        angle = std::fmod(angle, 2 * M_PI);
        if (angle > M_PI)
            angle -= 2 * M_PI;
        else if (angle <= -M_PI)
            angle += 2 * M_PI;
        return angle;
    };

    double th3 = phi - th1 - th2;
    th3 = normalizeRadians(th3);
    solutions.push_back(Eigen::Vector3d(th1, th2, th3));

    /*
     * Check if the target is on the edge of the workspace. If so, return only one solution. In the test cases,
     * we will check vector size to determine if the target is on the edge of the workspace.
     */

    /*if (endEffectorXYW[0] * endEffectorXYW[0] + endEffectorXYW[1] * endEffectorXYW[1] == maxReach * maxReach)
    {
        return solutions;
    }*/

    double th2_2 = -th2;
    double th1_2 = atan2(p_2y, p_2x) - atan2(linkLengths[1] * sin(th2_2), linkLengths[0] + linkLengths[1] * cos(th2_2));
    double th3_2 = phi - th1_2 - th2_2;
    th3_2 = normalizeRadians(th3_2);
    solutions.push_back(Eigen::Vector3d(th1_2, th2_2, th3_2));

    return solutions;
}

Eigen::Matrix3d ThreeDOFRobot::computeJacobian(const Eigen::Vector3d &theta)
{
    Eigen::Matrix3d J;

    // Derivatives with respect to theta1
    J(0, 0) = -linkLengths[0] * sin(theta(0)) - linkLengths[1] * sin(theta(0) + theta(1)) - linkLengths[2] * sin(theta(0) + theta(1) + theta(2));
    J(1, 0) = linkLengths[0] * cos(theta(0)) + linkLengths[1] * cos(theta(0) + theta(1)) + linkLengths[2] * cos(theta(0) + theta(1) + theta(2));
    J(2, 0) = 1; // Assuming phi = theta1 + theta2 + theta3

    // Derivatives with respect to theta2
    J(0, 1) = -linkLengths[1] * sin(theta(0) + theta(1)) - linkLengths[2] * sin(theta(0) + theta(1) + theta(2));
    J(1, 1) = linkLengths[1] * cos(theta(0) + theta(1)) + linkLengths[2] * cos(theta(0) + theta(1) + theta(2));
    J(2, 1) = 1; // Assuming phi = theta1 + theta2 + theta3

    // Derivatives with respect to theta3
    J(0, 2) = -linkLengths[2] * sin(theta(0) + theta(1) + theta(2));
    J(1, 2) = linkLengths[2] * cos(theta(0) + theta(1) + theta(2));
    J(2, 2) = 1; // Assuming phi = theta1 + theta2 + theta3

    return J;
}

Eigen::Vector3d ThreeDOFRobot::solveInverseKinematicsNR(const Eigen::Vector3d &desiredPosition)
{
    // Initial guess for the joint angles
    Eigen::Vector3d theta = getJointAngles();

    int maxIterations = 10000;
    double tolerance = 1e-6;
    Eigen::Vector3d error;
    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // Step 2: Calculate current end-effector position using forward kinematics
        setJointAngRadians({theta(0), theta(1), theta(2)});
        Eigen::Vector3d currentPos = solveForwardKinematicsDH();

        // Step 3: Compute the Jacobian matrix for the current joint angles
        Eigen::Matrix3d J = computeJacobian(theta);

        // Step 4: Calculate position error
        error = desiredPosition - currentPos;
        

        // Check for convergence
        if (error.norm() < tolerance)
        {
            std::cout << "Converged to solution in " << iter << " iterations." << std::endl;
            break;
        }

        // Step 5: Update joint angles using Newton-Raphson method
        Eigen::Vector3d deltaTheta = J.inverse() * error;
        theta += deltaTheta;
    }
    std::cout << "Error: " << error.norm() << std::endl;

    auto normalizeRadians = [](double angle)
    {
        angle = std::fmod(angle, 2 * M_PI);
        if (angle > M_PI)
            angle -= 2 * M_PI;
        else if (angle <= -M_PI)
            angle += 2 * M_PI;
        return angle;
    };
    theta(0) = normalizeRadians(theta(0));
    theta(1) = normalizeRadians(theta(1));
    theta(2) = normalizeRadians(theta(2));

    // Check for NaN in the solution
    if (std::isnan(theta(0)) || std::isnan(theta(1)) || std::isnan(theta(2)))
    {
        std::cerr << "Solution did not converge." << std::endl;
        // Handle the non-convergence case, e.g., by returning a vector of NaNs
        return Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN());
    }

    // std::cout << "Solution: " << theta.transpose() * 180 / M_PI << std::endl;
    return theta;
}
