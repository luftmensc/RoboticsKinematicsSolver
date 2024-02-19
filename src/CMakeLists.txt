# Add library target
add_library(ThreeDOFRobot SHARED
    ThreeDOFRRR.cpp
)

target_include_directories(ThreeDOFRobot
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(ThreeDOFRobot
    Eigen3::Eigen
)

# For executables in examples directory
add_executable(example ../examples/examples.cpp)
target_link_libraries(example ThreeDOFRobot)
