cmake_minimum_required(VERSION 3.10)
project(ThreeDOFRobot VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Eigen library include

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(include)

# Add the subdirectories
add_subdirectory(src)
add_subdirectory(tests)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.11.0 # Specify the desired version
)
FetchContent_MakeAvailable(googletest)

