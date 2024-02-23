# RoboticsKinematicsSolver
Forward and Inverse Kinematics Solver Library from Scratch
# ThreeDOFRobot Kinematics Solver

## Overview

The ThreeDOFRobot library is a comprehensive C++ solution for solving the kinematics of a 3-DOF (Degree of Freedom) RRR (Revolute-Revolute-Revolute) type robotic arm. This library allows users to set joint angles, link lengths, and calculate the position of the end effector through both forward and inverse kinematics approaches. Additionally, it includes functionality to check if the end effector is within a given circle, making it a versatile tool for robotics applications.

Leveraging the Eigen library for vector and matrix operations, the ThreeDOFRobot library ensures high performance and accuracy in kinematics calculations. It also integrates Python for runtime operations, utilizing matplotlib for plotting, thus providing a visually rich representation of the robotic arm's movements and positions.

This software works as a standalone library and can be easily integrated into other robotics projects, providing a robust and efficient solution for solving the kinematics of a 3-DOF robotic arm.

Also provides example code for the library usage, including Python integration for plotting the robotic arm's configuration. You can use the example and give the inputs from the terminal and use it like console application.

## Features

- **Forward Kinematics**: Solve using either the Denavit-Hartenberg (DH) parameters or direct linear algebra methods.
- **Inverse Kinematics**: Offers both algebraic and numerical optimization (Newton-Raphson) solutions, including handling multiple possible solutions.
- **End Effector Position Check**: Function to verify if the end effector is within a specified circle, based on its current position.
- **Python Integration**: Runs Python scripts at runtime for plotting with matplotlib, allowing for easy visualization of the robotic arm's configuration.

## Installation

- sudo apt-get install libeigen3-dev
- sudo apt-get install python3-matplotlib
- sudo apt-get install python3-pip
- pip3 install matplotlib


### Prerequisites

- CMake (version 3.10 or higher)
- Eigen3 (version 3.3 or higher)
- Python (version 3.8.10)
- matplotlib for Python plotting

### Building the Library

### Linux Installation

1. **Clone the Repository**

- git clone https://github.com/luftmensc/RoboticsKinematicsSolver.git   
- cd RoboticsKinematicsSolver
- mkdir build
- cd build
- cmake .. && make

2. **Run the Example Executable Program With Python Integration**
- ./example



