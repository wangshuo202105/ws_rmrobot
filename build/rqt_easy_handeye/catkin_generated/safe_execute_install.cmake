execute_process(COMMAND "/home/ws/ws_rmrobot/build/rqt_easy_handeye/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ws/ws_rmrobot/build/rqt_easy_handeye/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
