#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "../include/KinematicsLib/ThreeDOFRRR.hpp"


int main()
{

    std::unique_ptr<ThreeDOFRobot> robot = std::make_unique<ThreeDOFRobot>();

    robot->setJointAngRadians(Eigen::Vector3d(90, 90, 90) * M_PI / 180.0); // In radians
    robot->setLinkLengths(Eigen::Vector3d(1.0, 1.0, 1.0)); // Set link lengths

    Eigen::Vector3d endEffectorPosition = robot->solveForwardKinematicsDH();
    std::cout << "End effector position (DH): " << endEffectorPosition.transpose() << std::endl;

    endEffectorPosition = robot->solveForwardKinematicsLS();
    std::cout << "End effector position (LS): " << endEffectorPosition.transpose() << std::endl;

    std::cout<< "Is end effector in circle: " << robot->isEndEffectorInCircle(0.0, 0.0, 1, M_PI/2, M_PI/2, M_PI/2) << std::endl; //right on the circle test
    std::cout<< "Is end effector in circle: " << robot->isEndEffectorInCircle(0.0, 0.0, 0.9, M_PI/2, M_PI/2, M_PI/2) << std::endl; //outside the circle test

    // Inverse kinematics Test:
    Eigen::Vector3d endEffectorXYW = Eigen::Vector3d(0,33,M_PI/2);
    Eigen::Vector3d jointAngles = robot->solveInverseKinematics(endEffectorXYW);
    std::cout << "Inverse kinematics: " << jointAngles.transpose() << std::endl;


    return 0;
}