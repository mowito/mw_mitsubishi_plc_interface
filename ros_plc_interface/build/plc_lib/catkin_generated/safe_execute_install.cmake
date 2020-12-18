execute_process(COMMAND "/home/ankur/Documents/difacto_mowito/mw_mitsubishi_plc_interface/ros_plc_interface/build/plc_lib/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ankur/Documents/difacto_mowito/mw_mitsubishi_plc_interface/ros_plc_interface/build/plc_lib/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
