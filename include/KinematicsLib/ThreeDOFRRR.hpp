#ifndef THREE_DOF_RRR_HPP
#define THREE_DOF_RRR_HPP


#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>

class ThreeDOFRobot
{
public:
    ThreeDOFRobot(Eigen::Vector3d jointAngles, Eigen::Vector3d linkLengths);
    ThreeDOFRobot();
    virtual ~ThreeDOFRobot();

    void setJointAngRadians(const Eigen::Vector3d &angles);
    void setLinkLengths(const Eigen::Vector3d &lengths);
    Eigen::Vector3d getJointAngles(void) const;
    Eigen::Vector3d getLinkLengths(void) const;

    //task 1-2
    Eigen::Vector3d solveForwardKinematicsDH();
    Eigen::Vector3d solveForwardKinematicsLS(void);
    Eigen::Matrix4d getCalculatedTransformationMatrix(void);

    //task 3
    bool isEndEffectorInCircle(const double &circle_x, const double &circle_y, const double &r,
                               const double &th1, const double &th2, const double &th3);

    //task 4 algebraic solution
    Eigen::Vector3d solveInverseKinematics(const Eigen::Vector3d &endEffectorXYW);
    std::vector<Eigen::Vector3d> solveInverseKinematics2Solution(const Eigen::Vector3d &endEffectorXYW);

    //task 4 numerical optimization Newthon-Raphson Method solution
    Eigen::Matrix3d computeJacobian(const Eigen::Vector3d& theta);
    Eigen::Vector3d solveInverseKinematicsNR(const Eigen::Vector3d& desiredPosition);

private:
    Eigen::Vector3d jointAngles{10*M_PI/180, 20*M_PI/180, 30*M_PI/180}; //escaping singularity
    Eigen::Vector3d linkLengths{1, 1, 1};

    Eigen::Matrix4d calculatedTransformationMatrix; //Model of the robot in DH parameters task 1
    Eigen::Matrix4d calculateTransformationMatrix(const double &theta, const double &a, const double &alpha, const double &d);
};

#endif // THREE_DOF_RRR_HPP
