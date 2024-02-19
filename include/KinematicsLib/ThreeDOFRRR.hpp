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
    ThreeDOFRobot(void);
    ThreeDOFRobot(const std::vector<double> &jointAngles, const std::vector<double> &linkLengths);
    virtual ~ThreeDOFRobot();

    void setJointAngRadians(const Eigen::Vector3d &angles);
    void setLinkLengths(const Eigen::Vector3d &lengths);
    Eigen::Vector3d getJointAngles(void);
    Eigen::Vector3d getLinkLengths(void);

    //task 1-2
    Eigen::Vector3d solveForwardKinematicsDH();
    Eigen::Vector3d solveForwardKinematicsLS(void);
    Eigen::Matrix4d getCalculatedTransformationMatrix(void);

private:
    Eigen::Vector3d jointAngles;
    Eigen::Vector3d linkLengths;

    Eigen::Matrix4d calculatedTransformationMatrix;
    Eigen::Matrix4d calculateTransformationMatrix(const double &theta, const double &a, const double &alpha, const double &d);
};

#endif // THREE_DOF_RRR_HPP
