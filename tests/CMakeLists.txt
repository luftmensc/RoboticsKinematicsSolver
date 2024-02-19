enable_testing()

add_executable(test_main
    test_main.cpp
)

# Link the executable to the GoogleTest and ThreeDOFRobot libraries
target_link_libraries(test_main
    gtest_main
    ThreeDOFRobot
)

include(GoogleTest)
gtest_discover_tests(test_main)
