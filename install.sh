#!/bin/bash

# Stop script on any error
set -e

# Update and Upgrade Ubuntu Packages
echo "Updating Ubuntu packages..."
sudo apt-get update

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