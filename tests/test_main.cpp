#include <gtest/gtest.h>
#include "../include/KinematicsLib/ThreeDOFRRR.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <memory> 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct ForwardKinematicsTestParam {
    Eigen::Vector3d jointAngles; // Angles in degrees
    Eigen::Vector3d linkLengths;
    Eigen::Vector3d expectedPosition;
};

class ForwardKinematicsTest : public ::testing::TestWithParam<ForwardKinematicsTestParam> {};

// Parameterized test for checking forward kinematics
TEST_P(ForwardKinematicsTest, CalculatesCorrectPosition) {
    auto param = GetParam();
    auto robot = std::make_unique<ThreeDOFRobot>();

    Eigen::Vector3d jointAnglesRadians = param.jointAngles * M_PI / 180.0;
    robot->setJointAngRadians(jointAnglesRadians);
    robot->setLinkLengths(param.linkLengths);
    Eigen::Vector3d position = robot->solveForwardKinematicsDH();
    
    EXPECT_NEAR(position[0], param.expectedPosition[0], 1e-2);
    EXPECT_NEAR(position[1], param.expectedPosition[1], 1e-2);
    EXPECT_NEAR(position[2], param.expectedPosition[2], 1e-2);
}

INSTANTIATE_TEST_SUITE_P(
    ThreeDOFRobotTests,
    ForwardKinematicsTest,
    ::testing::Values(
        ForwardKinematicsTestParam{Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(3.0, 0.0, 0.0)},
        ForwardKinematicsTestParam{Eigen::Vector3d(90.0, 90.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(-2.0, 1.0, M_PI)},
        ForwardKinematicsTestParam{Eigen::Vector3d(30.0, 60.0, 90.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(-0.1339, 1.5, M_PI)}
    )
);

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
