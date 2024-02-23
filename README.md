
# Robotics Kinematics Solver
## Introduction
This document serves as a comprehensive guide for the Robotics Kinematics Solver, a specialized library designed for addressing the kinematics of a three-degree-of-freedom (3-DOF) Planar robotic arm characterized by a Revolute-Revolute-Revolute (RRR) joint configuration. The library's objective is to facilitate accurate and efficient kinematic analysis, incorporating both forward and inverse kinematics, alongside the capability to validate the end effector's position within specified parameters.
## Library Composition
The library is developed in C++, utilizing the Eigen library for sophisticated vector and matrix operations, thereby ensuring optimal performance and precision in computations. Furthermore, it integrates Python for executing runtime scripts, employing matplotlib for the graphical representation of the robotic arm's trajectory and positional states.

## Features

**Forward Kinematics**: Implements both the Denavit-Hartenberg (DH) parameter method and direct linear algebra approaches for calculating the position and orientation of the end effector.

**Inverse Kinematics**: Provides solutions through algebraic methods and numerical optimization techniques, such as the Newton-Raphson method, accommodating multiple potential solutions.

**End Effector Position Verification**: Includes a function to ascertain whether the end effector resides within a predefined circular area, enhancing the utility for specific robotic applications.

**Python Integration**: Facilitates the visualization of the robotic arm's configuration via matplotlib, enhancing the interpretability of its kinematic state.

## Installation and Setup

### Prerequisites

- g++ atleast 14
- CMake (version 3.10 or higher)
- Eigen3 (version 3.3 or higher)
- Python (version 3.8.10)
- matplotlib for Python plotting


- **For installation, users may execute the installation.sh script for automated setup. Alternatively, manual installation steps are provided below:**
```
sudo apt-get install cmake -y
sudo apt-get install g++ make -y
sudo apt-get install libeigen3-dev -y
sudo apt-get install python3-matplotlib
sudo apt-get install python3-pip
pip3 install matplotlib
```

### Building the Library

### Linux Installation

1. **Clone the Repository**
The source code is hosted on GitHub: [luftmensc/RoboticsKinematicsSolver](https://github.com/luftmensc/RoboticsKinematicsSolver).
```
git clone https://github.com/luftmensc/RoboticsKinematicsSolver.git
```
2. **Navigate to the Directory and Create a Build Folder**
```
cd RoboticsKinematicsSolver
mkdir build
cd build
```
3. **Build the Library**
```
cmake .. && make
```
4. **Run the Example to Verify the Installation**
```
./example
```

## GTest
Post-compilation, execute the test suite to confirm the library's integrity:
```
cd tests && ./test_main
```
## Documentation with Doxygen
**To generate comprehensive documentation, ensure Doxygen is installed and execute it within the docs directory using the provided Doxyfile:**:
```
sudo apt install doxygen
cd docs && doxygen
```
Then, navigate to the html directory and open the index.html file in a web browser to access the documentation. For instance, using Firefox:
```
cd html && firefox index.html
```


