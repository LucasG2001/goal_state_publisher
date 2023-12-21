# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "goal_state_publisher: 3 messages, 0 services")

set(MSG_I_FLAGS "-Igoal_state_publisher:/home/sktistakis/ros_ws/src/goal_state_publisher/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(goal_state_publisher_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg" NAME_WE)
add_custom_target(_goal_state_publisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "goal_state_publisher" "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg" ""
)

get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg" NAME_WE)
add_custom_target(_goal_state_publisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "goal_state_publisher" "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg" ""
)

get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg" NAME_WE)
add_custom_target(_goal_state_publisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "goal_state_publisher" "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_cpp(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_cpp(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/goal_state_publisher
)

### Generating Services

### Generating Module File
_generate_module_cpp(goal_state_publisher
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/goal_state_publisher
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(goal_state_publisher_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(goal_state_publisher_generate_messages goal_state_publisher_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_cpp _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_cpp _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_cpp _goal_state_publisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(goal_state_publisher_gencpp)
add_dependencies(goal_state_publisher_gencpp goal_state_publisher_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS goal_state_publisher_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_eus(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_eus(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/goal_state_publisher
)

### Generating Services

### Generating Module File
_generate_module_eus(goal_state_publisher
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/goal_state_publisher
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(goal_state_publisher_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(goal_state_publisher_generate_messages goal_state_publisher_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_eus _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_eus _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_eus _goal_state_publisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(goal_state_publisher_geneus)
add_dependencies(goal_state_publisher_geneus goal_state_publisher_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS goal_state_publisher_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_lisp(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_lisp(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/goal_state_publisher
)

### Generating Services

### Generating Module File
_generate_module_lisp(goal_state_publisher
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/goal_state_publisher
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(goal_state_publisher_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(goal_state_publisher_generate_messages goal_state_publisher_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_lisp _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_lisp _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_lisp _goal_state_publisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(goal_state_publisher_genlisp)
add_dependencies(goal_state_publisher_genlisp goal_state_publisher_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS goal_state_publisher_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_nodejs(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_nodejs(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/goal_state_publisher
)

### Generating Services

### Generating Module File
_generate_module_nodejs(goal_state_publisher
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/goal_state_publisher
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(goal_state_publisher_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(goal_state_publisher_generate_messages goal_state_publisher_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_nodejs _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_nodejs _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_nodejs _goal_state_publisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(goal_state_publisher_gennodejs)
add_dependencies(goal_state_publisher_gennodejs goal_state_publisher_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS goal_state_publisher_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_py(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/goal_state_publisher
)
_generate_msg_py(goal_state_publisher
  "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/goal_state_publisher
)

### Generating Services

### Generating Module File
_generate_module_py(goal_state_publisher
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/goal_state_publisher
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(goal_state_publisher_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(goal_state_publisher_generate_messages goal_state_publisher_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/GraspMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_py _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/MoveGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_py _goal_state_publisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sktistakis/ros_ws/src/goal_state_publisher/msg/StopGripperMsg.msg" NAME_WE)
add_dependencies(goal_state_publisher_generate_messages_py _goal_state_publisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(goal_state_publisher_genpy)
add_dependencies(goal_state_publisher_genpy goal_state_publisher_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS goal_state_publisher_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/goal_state_publisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/goal_state_publisher
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(goal_state_publisher_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/goal_state_publisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/goal_state_publisher
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(goal_state_publisher_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/goal_state_publisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/goal_state_publisher
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(goal_state_publisher_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/goal_state_publisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/goal_state_publisher
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(goal_state_publisher_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/goal_state_publisher)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/goal_state_publisher\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/goal_state_publisher
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(goal_state_publisher_generate_messages_py std_msgs_generate_messages_py)
endif()
