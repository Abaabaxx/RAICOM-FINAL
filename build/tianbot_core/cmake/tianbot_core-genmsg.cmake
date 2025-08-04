# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tianbot_core: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tianbot_core_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv" NAME_WE)
add_custom_target(_tianbot_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tianbot_core" "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tianbot_core
  "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tianbot_core
)

### Generating Module File
_generate_module_cpp(tianbot_core
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tianbot_core
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tianbot_core_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tianbot_core_generate_messages tianbot_core_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv" NAME_WE)
add_dependencies(tianbot_core_generate_messages_cpp _tianbot_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tianbot_core_gencpp)
add_dependencies(tianbot_core_gencpp tianbot_core_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tianbot_core_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(tianbot_core
  "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tianbot_core
)

### Generating Module File
_generate_module_eus(tianbot_core
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tianbot_core
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tianbot_core_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tianbot_core_generate_messages tianbot_core_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv" NAME_WE)
add_dependencies(tianbot_core_generate_messages_eus _tianbot_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tianbot_core_geneus)
add_dependencies(tianbot_core_geneus tianbot_core_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tianbot_core_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tianbot_core
  "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tianbot_core
)

### Generating Module File
_generate_module_lisp(tianbot_core
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tianbot_core
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tianbot_core_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tianbot_core_generate_messages tianbot_core_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv" NAME_WE)
add_dependencies(tianbot_core_generate_messages_lisp _tianbot_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tianbot_core_genlisp)
add_dependencies(tianbot_core_genlisp tianbot_core_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tianbot_core_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(tianbot_core
  "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tianbot_core
)

### Generating Module File
_generate_module_nodejs(tianbot_core
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tianbot_core
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tianbot_core_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tianbot_core_generate_messages tianbot_core_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv" NAME_WE)
add_dependencies(tianbot_core_generate_messages_nodejs _tianbot_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tianbot_core_gennodejs)
add_dependencies(tianbot_core_gennodejs tianbot_core_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tianbot_core_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tianbot_core
  "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tianbot_core
)

### Generating Module File
_generate_module_py(tianbot_core
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tianbot_core
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tianbot_core_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tianbot_core_generate_messages tianbot_core_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tianbot/tianbot_ws/src/tianbot_core/srv/DebugCmd.srv" NAME_WE)
add_dependencies(tianbot_core_generate_messages_py _tianbot_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tianbot_core_genpy)
add_dependencies(tianbot_core_genpy tianbot_core_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tianbot_core_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tianbot_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tianbot_core
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tianbot_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tianbot_core
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tianbot_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tianbot_core
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tianbot_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tianbot_core
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tianbot_core)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tianbot_core\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tianbot_core
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
