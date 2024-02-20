#include <gtest/gtest.h>
#include "../include/KinematicsLib/ThreeDOFRRR.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <memory> 

/*
* This is the test file for the ThreeDOFRRR class.
* All angles are in radians inside the test cases.
* Input test cases and expected results are in radians normalized between -pi and pi.
* (-180,180]  -180 not included, 180 included
*/

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

    Eigen::Vector3d jointAnglesRadians = param.jointAngles;
    robot->setJointAngRadians(jointAnglesRadians);
    robot->setLinkLengths(param.linkLengths);
    Eigen::Vector3d position = robot->solveForwardKinematicsDH();
    
    EXPECT_NEAR(position[0], param.expectedPosition[0], 1e-3);
    EXPECT_NEAR(position[1], param.expectedPosition[1], 1e-3);
    EXPECT_NEAR(position[2], param.expectedPosition[2], 1e-3);
}

INSTANTIATE_TEST_SUITE_P(
    ThreeDOFRobotTests,
    ForwardKinematicsTest,
    ::testing::Values(
        ForwardKinematicsTestParam{Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(3.0, 0.0, 0.0)},
        ForwardKinematicsTestParam{Eigen::Vector3d(M_PI/2,M_PI/2, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(-2.0, 1.0, M_PI)},
        ForwardKinematicsTestParam{Eigen::Vector3d(M_PI/6, M_PI/3, M_PI/2), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(-0.1339, 1.5, M_PI)},
        ForwardKinematicsTestParam{Eigen::Vector3d(M_PI/6, M_PI/3, -M_PI/2), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.866, 1.5, 0.0)}
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
    std::unique_ptr<ThreeDOFRobot> robot = std::make_unique<ThreeDOFRobot>();
    Eigen::Vector3d jointAngles = robot->solveInverseKinematics(param.endEffectorXYW); // link lengths are set to 1.0, 1.0, 1.0 in the constructor
    
    EXPECT_NEAR(jointAngles[0], param.expectedJointAngles[0], 1e-3);
    EXPECT_NEAR(jointAngles[1], param.expectedJointAngles[1], 1e-3);
    EXPECT_NEAR(jointAngles[2], param.expectedJointAngles[2], 1e-3);
}

INSTANTIATE_TEST_SUITE_P(
    InverseKinematicsTests,
    InverseKinematicsTest,
    ::testing::Values(
        InverseKinematicsTestParam{Eigen::Vector3d(3.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)},
        InverseKinematicsTestParam{Eigen::Vector3d(1.866, 1.5, 0.0), Eigen::Vector3d(M_PI/6, M_PI/3,-M_PI/2)},
        InverseKinematicsTestParam{Eigen::Vector3d(-1.7071,0.292,(-135*M_PI/180)), Eigen::Vector3d(M_PI/2, M_PI/2, M_PI/4)}
    )
);
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
