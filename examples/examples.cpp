#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "../include/KinematicsLib/ThreeDOFRRR.hpp"


int main()
{

    std::unique_ptr<ThreeDOFRobot> robot = std::make_unique<ThreeDOFRobot>();

    //robot->setJointAngRadians(Eigen::Vector3d(30, 60, -90) * M_PI / 180.0); // In radians
    robot->setJointAngRadians(Eigen::Vector3d(90,90,45) * M_PI / 180.0); // In radians
    robot->setLinkLengths(Eigen::Vector3d(1.0, 1.0, 1.0)); // Set link lengths

    Eigen::Vector3d endEffectorPosition = robot->solveForwardKinematicsDH();
    std::cout << "\nEnd effector position (DH): " << endEffectorPosition.transpose() << " - Input joint angles: " << robot->getJointAngles().transpose() << std::endl;

    endEffectorPosition = robot->solveForwardKinematicsLS();
    std::cout << "End effector position (LS): " << endEffectorPosition.transpose() << " - Input joint angles: " << robot->getJointAngles().transpose() << std::endl;
/*
    std::cout<< "Is end effector in circle: " << robot->isEndEffectorInCircle(0.0, 0.0, 1, M_PI/2, M_PI/2, M_PI/2) << std::endl; //right on the circle test
    std::cout<< "Is end effector in circle: " << robot->isEndEffectorInCircle(0.0, 0.0, 0.9, M_PI/2, M_PI/2, M_PI/2) << std::endl; //outside the circle test
*/
    // Inverse kinematics Test:
    Eigen::Vector3d endEffectorXYW = Eigen::Vector3d(-1, 0 ,M_PI/4);
    Eigen::Vector3d jointAngles = robot->solveInverseKinematics(endEffectorXYW);
    std::cout<<"While end effector is at: "<<endEffectorXYW.transpose()<<std::endl;
    std::cout << "\nInverse kinematics: " << jointAngles.transpose() << " in degrees: " << jointAngles.transpose() * 180.0 / M_PI << " - Input end effector position: " << endEffectorXYW.transpose() << std::endl;
    std::vector<Eigen::VectorXd> jointAngles2 = robot->solveInverseKinematics2Solution(endEffectorXYW);
    std::cout << "Inverse kinematics with 2 solutions, \n\nfirst:  " << jointAngles2[0].transpose() << " in degrees: " << jointAngles2[0].transpose() * 180.0 / M_PI << "\nSecnd: " << jointAngles2[1].transpose() << " in degrees: " << jointAngles2[1].transpose() * 180.0 / M_PI << "\n\n";

    return 0;
}