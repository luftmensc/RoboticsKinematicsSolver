/**
 * @file ThreeDOFRRR.hpp
 * @author Ömer (omerokuyan7034@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */


/**
 * @file ThreeDOFRobot.hpp
 * @brief Defines the ThreeDOFRobot class for solving kinematics of a 3-DOF RRR planar robotic arm.
 */

#ifndef THREE_DOF_RRR_HPP
#define THREE_DOF_RRR_HPP


#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>

/**
 * @class ThreeDOFRobot
 * @brief This class models a 3-DOF (Degree of Freedom) RRR (Revolute-Revolute-Revolute) type robotic arm.
 *
 * It provides functionalities to set and get joint angles, link lengths, and the end-effector position.
 * Additionally, it offers methods to solve forward and inverse kinematics in multiple ways.
 */

class ThreeDOFRobot
{
public:
    /**
     * @brief Construct a new Three D O F Robot object
     * 
     * @param jointAngles 
     * @param linkLengths 
     */
    ThreeDOFRobot(Eigen::Vector3d jointAngles, Eigen::Vector3d linkLengths);

    /**
     * @brief Default constructor.
     */
    ThreeDOFRobot();
    /**
     * @brief Virtual destructor.
     */
    virtual ~ThreeDOFRobot();
    
    /**
     * @brief function to set the joint angles in radians.
     * @param angles in radian as Eigen::Vector3d.
     */
    void setJointAngRadians(const Eigen::Vector3d &angles);

    /**
     * @brief function to set the link lengths in meters.
     * @param angles in degrees as Eigen::Vector3d.
     */
    void setLinkLengths(const Eigen::Vector3d &lengths);

    /**
     * @brief function to set the end effector position
     * @param position in meters as Eigen::Vector3d.
     */
    void setEndEffectorPosition(const Eigen::Vector3d &position);

    /**
     * @brief returns the joint angles in radians as Eigen::Vector3d.
     * @return joint angles in radians as Eigen::Vector3d.
     */
    Eigen::Vector3d getJointAngles(void) const;

    /**
     * @brief returns the link lengths in meters as Eigen::Vector3d.
     * @return link lengths in meters as Eigen::Vector3d.
     */
    Eigen::Vector3d getLinkLengths(void) const;

    /**
     * @brief returns the end effector position in meters as Eigen::Vector3d.
     * @return end effector position in meters as Eigen::Vector3d.
     */
    Eigen::Vector3d getEndEffectorPosition(void) const;

    /**
     * @brief solveForwardKinematicsDH function is used to solve the forward kinematics using DH model parameters. Need to set the joint angles and link lengths before calling this function.
     * @return Returns the end effector position in Eigen::Vector3d.
     */
    Eigen::Vector3d solveForwardKinematicsDH();

    /**
     * @brief solveForwardKinematicsLS function is used to solve the forward kinematics using Direct Linear Algebra model parameters. Need to set the joint angles and link lengths before calling this function.
     * @return Returns the end effector position in Eigen::Vector3d.
    */
    Eigen::Vector3d solveForwardKinematicsLS(void);

    //task 3
    /**
     * @brief For given circle parameters and end effector position, isEndEffectorInCircle function checks whether the end effector is in the circle or not.
     * @param circle_x, circle_y, r, angle1, angle2, angle3 are the circle parameters and joint angles.
     * @return bool value, true if end effector is in the circle, false otherwise.
    **/
    bool isEndEffectorInCircle(const double &circle_x, const double &circle_y, const double &r,
                               const double &th1, const double &th2, const double &th3);

    //task 4 algebraic solution
    /**
     * @brief solveInverseKinematics function is used to solve the inverse kinematics using algebraic approach.
     * @param endEffectorXYW is the desired end effector position.
     * @return Returns the joint angles in Eigen::Vector3d.
    */
    Eigen::Vector3d solveInverseKinematics(const Eigen::Vector3d &endEffectorXYW);

    /**
     * @brief solveInverseKinematics2Solution function is used to solve the inverse kinematics using algebraic approach.
     * Since the robot has 2 solutions for the given end effector position, it returns the joint angles in Eigen::Vector3d.
     * @param endEffectorXYW is the desired end effector position.
     * @return Returns the joint angles in Eigen::Vector3d.
    */
    std::vector<Eigen::Vector3d> solveInverseKinematics2Solution(const Eigen::Vector3d &endEffectorXYW);

    //task 4 numerical optimization Newthon-Raphson Method solution
    /**
     * @brief solveInverseKinematicsNR function is used to solve the inverse kinematics using numerical optimization (Newton-Raphson) approach.
     * @param desiredPosition is the desired end effector position.
     * @return Returns the joint angles in Eigen::Vector3d.
    */
    
    Eigen::Vector3d solveInverseKinematicsNR(const Eigen::Vector3d& desiredPosition);

private:
    Eigen::Vector3d jointAngles{10*M_PI/180, 20*M_PI/180, 30*M_PI/180}; //escaping singularity
    Eigen::Vector3d linkLengths{1, 1, 1};
    Eigen::Vector3d endEffectorPosition;

    Eigen::Matrix4d calculatedTransformationMatrix; //Model of the robot in DH parameters task 1
    Eigen::Matrix4d calculateTransformationMatrix(const double &theta, const double &a, const double &alpha, const double &d);
    Eigen::Matrix3d computeJacobian(const Eigen::Vector3d& theta);
};

#endif // THREE_DOF_RRR_HPP
