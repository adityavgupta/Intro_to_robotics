execute_process(COMMAND "/home/ur3/catkin_avgupta3/build/drivers/universal_robot/ur_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ur3/catkin_avgupta3/build/drivers/universal_robot/ur_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
