#include <gtest/gtest.h>
#include "../include/KinematicsLib/ThreeDOFRRR.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <memory> 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct ForwardKinematicsTestParam {
    Eigen::Vector3d jointAngles;
    Eigen::Vector3d linkLengths;
    Eigen::Vector3d expectedPosition;
};

struct CircleTestParam {
    Eigen::Vector3d jointAnglesRadians; 
    double circle_x, circle_y, radius;
    bool expectedIsInCircle;
};

struct InverseKinematicsTestParam {
    Eigen::Vector3d endEffectorXYW;
    Eigen::Vector3d expectedJointAngles;
};

class ForwardKinematicsTest : public ::testing::TestWithParam<ForwardKinematicsTestParam> {};

class EndEffectorCircleTest : public ::testing::TestWithParam<CircleTestParam> {};

class InverseKinematicsTest : public ::testing::TestWithParam<InverseKinematicsTestParam> {};


// Parameterized test for checking forward kinematics
TEST_P(ForwardKinematicsTest, ForwardKinematicsTest) {
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

TEST_P(EndEffectorCircleTest, EndEffectorCircleTest) {
    auto param = GetParam();
    ThreeDOFRobot robot;
    robot.setJointAngRadians(param.jointAnglesRadians);
    bool isInCircle = robot.isEndEffectorInCircle(param.circle_x, param.circle_y, param.radius,
                                                  param.jointAnglesRadians[0], param.jointAnglesRadians[1], param.jointAnglesRadians[2]);
    
    EXPECT_EQ(isInCircle, param.expectedIsInCircle);
}

INSTANTIATE_TEST_SUITE_P(
    EndEffectorCircleTests,
    EndEffectorCircleTest,
    ::testing::Values(
        CircleTestParam{Eigen::Vector3d(M_PI/6, M_PI/4, M_PI/3), 2.0, 2.0, 3.0, true},
        CircleTestParam{Eigen::Vector3d(M_PI/2, M_PI/2, M_PI/2), 10.0, 10.0, 5.0, false}
    )
);

TEST_P(InverseKinematicsTest, InverseKinematicsTest) {
    auto param = GetParam();
    ThreeDOFRobot robot;
    Eigen::Vector3d jointAngles = robot.solveInverseKinematics(param.endEffectorXYW);
    
    EXPECT_NEAR(jointAngles[0], param.expectedJointAngles[0], 1e-2);
    EXPECT_NEAR(jointAngles[1], param.expectedJointAngles[1], 1e-2);
    EXPECT_NEAR(jointAngles[2], param.expectedJointAngles[2], 1e-2);
}

INSTANTIATE_TEST_SUITE_P(
    InverseKinematicsTests,
    InverseKinematicsTest,
    ::testing::Values(
        InverseKinematicsTestParam{Eigen::Vector3d(0.0, 0.0, 0), Eigen::Vector3d(0.0, 0.0, 0)}
    )
);
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


