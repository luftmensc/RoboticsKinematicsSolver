if(EXISTS "/home/omer/robotics_ws/build/tests/test_main[1]_tests.cmake")
  include("/home/omer/robotics_ws/build/tests/test_main[1]_tests.cmake")
else()
  add_test(test_main_NOT_BUILT test_main_NOT_BUILT)
endif()