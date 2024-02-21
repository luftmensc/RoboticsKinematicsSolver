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


