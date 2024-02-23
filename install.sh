#!/bin/bash

# Stop script on any error
set -e
echo "Installing required packages for Robotics Kinematics Solver..."
echo "cmake, g++, make, libeigen3-dev, python3, pip3, matplotlib, numpy will be installed."
echo "Hit any key to continue or CTRL+C to exit."

read -n 1 -s

# Update and Upgrade Ubuntu Packages
echo "Updating Ubuntu packages..."
sudo apt-get update

echo "Installing CMake..."
sudo apt-get install cmake -y

# Install Eigen3 for C++ (Eigen::Dense) if not already installed
echo "Installing Eigen3 library for C++..."
sudo apt-get install libeigen3-dev -y

# Install C++ compiler and make (g++, make) if not already installed
echo "Installing g++ and make..."
sudo apt-get install g++ make -y

# Install Python3 and pip if not already installed
echo "Installing Python3 and pip..."
sudo apt-get install python3 python3-pip -y

# Install Python packages
echo "Installing required Python packages..."
pip3 install matplotlib numpy


echo "Installation complete."