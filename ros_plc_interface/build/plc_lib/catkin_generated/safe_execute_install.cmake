execute_process(COMMAND "/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/build/plc_lib/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/up/mw_seimens_s7_plc_interface/ros_plc_interface/build/plc_lib/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
