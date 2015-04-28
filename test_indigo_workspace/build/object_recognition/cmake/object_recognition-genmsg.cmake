# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "object_recognition: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(object_recognition_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv" NAME_WE)
add_custom_target(_object_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_recognition" "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(object_recognition
  "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition
)

### Generating Module File
_generate_module_cpp(object_recognition
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(object_recognition_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(object_recognition_generate_messages object_recognition_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv" NAME_WE)
add_dependencies(object_recognition_generate_messages_cpp _object_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_recognition_gencpp)
add_dependencies(object_recognition_gencpp object_recognition_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_recognition_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(object_recognition
  "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition
)

### Generating Module File
_generate_module_lisp(object_recognition
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(object_recognition_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(object_recognition_generate_messages object_recognition_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv" NAME_WE)
add_dependencies(object_recognition_generate_messages_lisp _object_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_recognition_genlisp)
add_dependencies(object_recognition_genlisp object_recognition_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_recognition_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(object_recognition
  "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition
)

### Generating Module File
_generate_module_py(object_recognition
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(object_recognition_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(object_recognition_generate_messages object_recognition_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bryant/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition/srv/Image.srv" NAME_WE)
add_dependencies(object_recognition_generate_messages_py _object_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_recognition_genpy)
add_dependencies(object_recognition_genpy object_recognition_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_recognition_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(object_recognition_generate_messages_cpp object_recognition_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(object_recognition_generate_messages_lisp object_recognition_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(object_recognition_generate_messages_py object_recognition_generate_messages_py)
