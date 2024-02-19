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


    return 0;
}