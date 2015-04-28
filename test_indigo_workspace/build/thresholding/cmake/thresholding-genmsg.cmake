# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "thresholding: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(thresholding_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv" NAME_WE)
add_custom_target(_thresholding_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thresholding" "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(thresholding
  "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thresholding
)

### Generating Module File
_generate_module_cpp(thresholding
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thresholding
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(thresholding_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(thresholding_generate_messages thresholding_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv" NAME_WE)
add_dependencies(thresholding_generate_messages_cpp _thresholding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thresholding_gencpp)
add_dependencies(thresholding_gencpp thresholding_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thresholding_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(thresholding
  "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thresholding
)

### Generating Module File
_generate_module_lisp(thresholding
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thresholding
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(thresholding_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(thresholding_generate_messages thresholding_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv" NAME_WE)
add_dependencies(thresholding_generate_messages_lisp _thresholding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thresholding_genlisp)
add_dependencies(thresholding_genlisp thresholding_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thresholding_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(thresholding
  "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thresholding
)

### Generating Module File
_generate_module_py(thresholding
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thresholding
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(thresholding_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(thresholding_generate_messages thresholding_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/thresholding/srv/ProcessImage.srv" NAME_WE)
add_dependencies(thresholding_generate_messages_py _thresholding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thresholding_genpy)
add_dependencies(thresholding_genpy thresholding_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thresholding_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thresholding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thresholding
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(thresholding_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thresholding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thresholding
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(thresholding_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thresholding)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thresholding\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thresholding
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(thresholding_generate_messages_py std_msgs_generate_messages_py)
