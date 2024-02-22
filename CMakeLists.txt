cmake_minimum_required(VERSION 3.10)
project(ThreeDOFRobot VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Eigen library include

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(include)

# For executables in root directory and target_link_libraries
add_executable(example examples.cpp)
target_link_libraries(example ThreeDOFRobot)

# Add the subdirectories
add_subdirectory(src)
add_subdirectory(tests)

# Copy Python script to build directory after the build is complete
add_custom_command(
  TARGET example POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_SOURCE_DIR}/visualize_robot.py
          ${CMAKE_CURRENT_BINARY_DIR}/visualize_robot.py
  COMMENT "Copying visualize_robot.py to build directory"
)


include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.11.0 # Specify the desired version
)
FetchContent_MakeAvailable(googletest)

