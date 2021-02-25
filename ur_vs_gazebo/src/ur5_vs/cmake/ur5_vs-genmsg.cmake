# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ur5_vs: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iur5_vs:/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ur5_vs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg" NAME_WE)
add_custom_target(_ur5_vs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur5_vs" "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg" "std_msgs/Bool"
)

get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg" NAME_WE)
add_custom_target(_ur5_vs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur5_vs" "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg" "std_msgs/Float64"
)

get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg" NAME_WE)
add_custom_target(_ur5_vs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur5_vs" "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg" "std_msgs/Float64"
)

get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg" NAME_WE)
add_custom_target(_ur5_vs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur5_vs" "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg" "std_msgs/Float64"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_vs
)
_generate_msg_cpp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_vs
)
_generate_msg_cpp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_vs
)
_generate_msg_cpp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_vs
)

### Generating Services

### Generating Module File
_generate_module_cpp(ur5_vs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_vs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ur5_vs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ur5_vs_generate_messages ur5_vs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_cpp _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_cpp _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_cpp _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_cpp _ur5_vs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_vs_gencpp)
add_dependencies(ur5_vs_gencpp ur5_vs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_vs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_vs
)
_generate_msg_eus(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_vs
)
_generate_msg_eus(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_vs
)
_generate_msg_eus(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_vs
)

### Generating Services

### Generating Module File
_generate_module_eus(ur5_vs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_vs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ur5_vs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ur5_vs_generate_messages ur5_vs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_eus _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_eus _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_eus _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_eus _ur5_vs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_vs_geneus)
add_dependencies(ur5_vs_geneus ur5_vs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_vs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_vs
)
_generate_msg_lisp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_vs
)
_generate_msg_lisp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_vs
)
_generate_msg_lisp(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_vs
)

### Generating Services

### Generating Module File
_generate_module_lisp(ur5_vs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_vs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ur5_vs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ur5_vs_generate_messages ur5_vs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_lisp _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_lisp _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_lisp _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_lisp _ur5_vs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_vs_genlisp)
add_dependencies(ur5_vs_genlisp ur5_vs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_vs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_vs
)
_generate_msg_nodejs(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_vs
)
_generate_msg_nodejs(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_vs
)
_generate_msg_nodejs(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_vs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ur5_vs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_vs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ur5_vs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ur5_vs_generate_messages ur5_vs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_nodejs _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_nodejs _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_nodejs _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_nodejs _ur5_vs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_vs_gennodejs)
add_dependencies(ur5_vs_gennodejs ur5_vs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_vs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs
)
_generate_msg_py(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs
)
_generate_msg_py(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs
)
_generate_msg_py(ur5_vs
  "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs
)

### Generating Services

### Generating Module File
_generate_module_py(ur5_vs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ur5_vs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ur5_vs_generate_messages ur5_vs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/sim_variables.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_py _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_angles.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_py _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_vel.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_py _ur5_vs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/shaunak/ur_vs_gazebo/src/ur5_vs/msgs/joint_states.msg" NAME_WE)
add_dependencies(ur5_vs_generate_messages_py _ur5_vs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_vs_genpy)
add_dependencies(ur5_vs_genpy ur5_vs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_vs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_vs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_vs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ur5_vs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_vs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_vs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ur5_vs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_vs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_vs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ur5_vs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_vs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_vs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ur5_vs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_vs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ur5_vs_generate_messages_py std_msgs_generate_messages_py)
endif()
