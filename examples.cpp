#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <sstream> // Include for std::istringstream
#include <cstdlib> // For std::system

#include "include/KinematicsLib/ThreeDOFRRR.hpp"

int main()
{
    int action(0);

    while(action!=7){
        std::unique_ptr<ThreeDOFRobot> robot = std::make_unique<ThreeDOFRobot>();
        robot->setLinkLengths(Eigen::Vector3d(1.0, 1.0, 1.0)); // Set link lengths
        std::cout<<"You have 3 DoF revolute joint planar robot with 3 links of length 1.0m \n\n";

        std::cout << "Please specify the action you want to perform:\n\n1- Forward Kinematics with DH model\n2- Forward Kinematics with Algebraic Function\n"
        << "3- Inverse Kinematics Algebraic solution (Returns both elbow up-down solutions)\n4- Inverse Kinematics Algebraic solution (Returns only one solution)\n"
        << "5- Inverse Kinematics using numetical method (Newton-Raphson)\n6- Is in The Circle Test\n7- Quit\n\n";
        
        std::vector<double> CircleParams;
    
        std::cin >> action;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the input buffer
        std::cout<< "action: "<<action<<std::endl;

        if(!(action > 0 && action < 8)){
            std::cout<< "Invalid action, please input a number between 1 and 5"<<std::endl;
            std::cout<< "Error code: 1"<<std::endl;
            return 1;
        }
        if(action==1 || action==2){
            std::cout<< "Please input the joint angles in degrees (3 angles separated by space) to calculate FK"<<std::endl;
            std::string input;
            std::getline(std::cin, input);

            std::istringstream iss(input);
            std::vector<double> jointAngles;
            double angle;
            while(iss >> angle){
                jointAngles.push_back(angle);
            }
            if(jointAngles.size()!=3){
                std::cout<< "Invalid input, please input 3 joint angles"<<std::endl;
                std::cout<< "Error code: 2"<<std::endl;
                return 2;
            }
            robot->setJointAngRadians(Eigen::Vector3d(jointAngles[0], jointAngles[1], jointAngles[2]) * M_PI / 180.0); // In radians

        }
        else if (action==3 || action==4 || action==5){
            std::cout<< "Please input the end effector position (x, y, w) to calculate IK. Give the orientation angle in degrees!"<<std::endl;
            std::string input;
            std::getline(std::cin, input);
            
            std::istringstream iss(input);
            std::vector<double> endEffectorPosition;
            double value;
            while(iss >> value){
                endEffectorPosition.push_back(value);
            }
            if(endEffectorPosition.size()!=3){
                std::cout<< "Invalid input, please input 3 values for end effector position"<<std::endl;
                std::cout<< "Error code: 3"<<std::endl;
                return 3;
            }
            robot->setEndEffectorPosition(Eigen::Vector3d(endEffectorPosition[0], endEffectorPosition[1], endEffectorPosition[2] * M_PI / 180.0)); // In radians
        }
        else if(action==6){
            std::cout<< "Please input the circle parameters (x, y, r) and robot joint angles (3 angles separated by space) to check if the end effector is in the circle"<<std::endl;
            std::string input;
            std::getline(std::cin, input);

            std::istringstream iss(input);
            
            double value;
            while(iss >> value){
                CircleParams.push_back(value);
            }
            if(CircleParams.size()!=6){
                std::cout<< "Invalid input, please input 6 values for circle and end effector position"<<std::endl;
                std::cout<< "Error code: 4"<<std::endl;
                return 4;
            }
            robot->setJointAngRadians(Eigen::Vector3d(CircleParams[3], CircleParams[4], CircleParams[5]) * M_PI / 180.0); // In radians
        }

        else if(action==7){
            return 0;
        }
        else{
            std::cout<< "Invalid action, please input a number between 1 and 7"<<std::endl;
            std::cout<< "Error code: 5"<<std::endl;
            return 5;
        }
        std::cout << "\n\n";
        switch (action)
        {
        case 1: //FK DH
        {
            Eigen::Vector3d endEffectorPosition = robot->solveForwardKinematicsDH();
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::cout << "For the input joint angles (in radians): " << robot->getJointAngles().transpose() << std::endl;
            std::cout << "End effector position (DH): " << endEffectorPosition.transpose() << " - Input joint angles (in radians): " << robot->getJointAngles().transpose() << std::endl;
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;

            std::string command = "python3 visualize_robot.py " +
                                std::to_string(endEffectorPosition[0]) + " " + // X position of the end effector
                                std::to_string(endEffectorPosition[1]) + " " + // Y position of the end effector
                                std::to_string(robot->getJointAngles()[0]*180.0/M_PI) + " " + // Angle 1 in degrees
                                std::to_string(robot->getJointAngles()[1]*180.0/M_PI) + " " + // Angle 2 in degrees
                                std::to_string(robot->getJointAngles()[2]*180.0/M_PI);        // Angle 3 in degrees
            std::system(command.c_str());
            break;
        }
        case 2: //FK LS
        {
            Eigen::Vector3d endEffectorPosition = robot->solveForwardKinematicsLS();
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::cout << "End effector position (LS): " << endEffectorPosition.transpose() << " - Input joint angles (in radians): " << robot->getJointAngles().transpose() << std::endl;
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;

            std::string command = "python3 visualize_robot.py " +
                                std::to_string(endEffectorPosition[0]) + " " + // X position of the end effector
                                std::to_string(endEffectorPosition[1]) + " " + // Y position of the end effector
                                std::to_string(robot->getJointAngles()[0]*180.0/M_PI) + " " + // Angle 1 in degrees
                                std::to_string(robot->getJointAngles()[1]*180.0/M_PI) + " " + // Angle 2 in degrees
                                std::to_string(robot->getJointAngles()[2]*180.0/M_PI);        // Angle 3 in degrees
            std::system(command.c_str());

            break;
        }
        case 3: //IK algebraic 2 solutions
        {
            std::vector<Eigen::Vector3d> jointAngles = robot->solveInverseKinematics2Solution(robot->getEndEffectorPosition());
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::cout << "While end effector is at: "<<robot->getEndEffectorPosition().transpose()<<std::endl;
            std::cout << "Inverse kinematics solution 1: " << jointAngles[0].transpose() << " in degrees: " << jointAngles[0].transpose() * 180.0 / M_PI << std::endl;
            std::cout << "Inverse kinematics solution 2: " << jointAngles[1].transpose() << " in degrees: " << jointAngles[1].transpose() * 180.0 / M_PI << std::endl;
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::string command = "python3 visualize_robot.py " + 
                                std::to_string(robot->getEndEffectorPosition()[0]) + " " +
                                std::to_string(robot->getEndEffectorPosition()[1]) + " " +
                                std::to_string(jointAngles[0][0]*180.0/M_PI) + " " + 
                                std::to_string(jointAngles[0][1]*180.0/M_PI) + " " + 
                                std::to_string(jointAngles[0][2]*180.0/M_PI) + " " +
                                std::to_string(jointAngles[1][0]*180.0/M_PI) + " " + 
                                std::to_string(jointAngles[1][1]*180.0/M_PI) + " " + 
                                std::to_string(jointAngles[1][2]*180.0/M_PI);

            std::cout << "Given command to python: " << command << std::endl; 
            std::system(command.c_str());

            break;
        }
        case 4: //IK algebraic 1 solution
        {
            Eigen::Vector3d jointAngles = robot->solveInverseKinematics(robot->getEndEffectorPosition());
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::cout << "While end effector is at: "<<robot->getEndEffectorPosition().transpose()<<std::endl;
            std::cout << "Inverse kinematics: " << jointAngles.transpose() << " in degrees: " << jointAngles.transpose() * 180.0 / M_PI << std::endl;
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;

            std::string command = "python3 visualize_robot.py " + 
                    std::to_string(robot->getEndEffectorPosition()[0]) + " " +
                    std::to_string(robot->getEndEffectorPosition()[1]) + " " +
                    std::to_string(jointAngles[0]*180.0/M_PI) + " " +
                    std::to_string(jointAngles[1]*180.0/M_PI) + " " +
                    std::to_string(jointAngles[2]*180.0/M_PI); 

            std::cout << "Given command to python: " << command << std::endl; 
            std::system(command.c_str());
            break;
        }
        case 5: //IK NR
        {
            std::cout<<"Do you want to input initial guess for Newton-Raphson method? \nIf you select no, the robot's last position will be used as the initial guess.(y/n)"<<std::endl;
            char input;
            std::cin >> input;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the input buffer

            if(input=='y'){
                std::cout<< "Please input the initial angle (in degrees) guess for Newton-Raphson method (3 angles separated by space)"<<std::endl;
                std::string input;
                std::getline(std::cin, input);

                std::istringstream iss(input);
                std::vector<double> initialGuess;
                double angle;
                while(iss >> angle){
                    initialGuess.push_back(angle);
                }
                if(initialGuess.size()!=3){
                    std::cout<< "Invalid input, please input 3 joint angles"<<std::endl;
                    std::cout<< "Error code: 6"<<std::endl;
                    return 6;
                }
                robot->setJointAngRadians(Eigen::Vector3d(initialGuess[0], initialGuess[1], initialGuess[2]) * M_PI / 180.0); // In radians
            }
            else if(input=='n'){
                std::cout<< "Using the last position as the initial guess for Newton-Raphson method"<<std::endl;
            }
            else{
                std::cout<< "Invalid input, please input y or n"<<std::endl;
                std::cout<< "Error code: 7"<<std::endl;
                return 7;
            }
            Eigen::Vector3d jointAnglesNR = robot->solveInverseKinematicsNR(robot->getEndEffectorPosition());
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::cout << "Inverse kinematics using Newton-Raphson: " << jointAnglesNR.transpose() << " in degrees: " << jointAnglesNR.transpose() * 180.0 / M_PI << std::endl;
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::string command = "python3 visualize_robot.py " + 
                    std::to_string(robot->getEndEffectorPosition()[0]) + " " +
                    std::to_string(robot->getEndEffectorPosition()[1]) + " " +
                    std::to_string(jointAnglesNR[0]*180.0/M_PI) + " " +
                    std::to_string(jointAnglesNR[1]*180.0/M_PI) + " " +
                    std::to_string(jointAnglesNR[2]*180.0/M_PI);
            std::system(command.c_str());

            break;
        }
        case 6: //Is in the circle
        {   
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            if(robot->isEndEffectorInCircle(CircleParams[0], CircleParams[1], CircleParams[2], robot->getJointAngles()[0], robot->getJointAngles()[1], robot->getJointAngles()[2])){
                
                std::cout<< "End effector is in the circle"<<std::endl;
            }
            else{
                std::cout<< "End effector is not in the circle"<<std::endl;
            }
            std::cout << "------------------------------------------------------------------------------------------------"<<std::endl;
            std::string command = "python3 visualize_robot.py" + 
                    std::to_string(CircleParams[0]) + " " +
                    std::to_string(CircleParams[1]) + " " +
                    std::to_string(CircleParams[2]) + " " +
                    std::to_string(robot->getEndEffectorPosition()[0]) + " " +
                    std::to_string(robot->getEndEffectorPosition()[1]) + " " +
                    std::to_string(robot->getEndEffectorPosition()[2]) + " " +
                    std::to_string(robot->getJointAngles()[0]*180.0/M_PI) + " " +
                    std::to_string(robot->getJointAngles()[1]*180.0/M_PI) + " " +
                    std::to_string(robot->getJointAngles()[2]*180.0/M_PI);
            std::system(command.c_str());
                    
            break;
        }
        default:
            break;
        }
        std::cout << "\n\n";
    }

    return 0;
}