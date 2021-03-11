# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "frame_tracking: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iframe_tracking:/home/milab/catkin_ws/src/frame_tracking/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iframe_tracking:/home/milab/catkin_ws/src/frame_tracking/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(frame_tracking_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg" NAME_WE)
add_custom_target(_frame_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "frame_tracking" "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg" ""
)

get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg" NAME_WE)
add_custom_target(_frame_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "frame_tracking" "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg" "frame_tracking/pointInformation"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frame_tracking
)
_generate_msg_cpp(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg"
  "${MSG_I_FLAGS}"
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frame_tracking
)

### Generating Services

### Generating Module File
_generate_module_cpp(frame_tracking
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frame_tracking
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(frame_tracking_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(frame_tracking_generate_messages frame_tracking_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_cpp _frame_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_cpp _frame_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frame_tracking_gencpp)
add_dependencies(frame_tracking_gencpp frame_tracking_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frame_tracking_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frame_tracking
)
_generate_msg_eus(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg"
  "${MSG_I_FLAGS}"
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frame_tracking
)

### Generating Services

### Generating Module File
_generate_module_eus(frame_tracking
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frame_tracking
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(frame_tracking_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(frame_tracking_generate_messages frame_tracking_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_eus _frame_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_eus _frame_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frame_tracking_geneus)
add_dependencies(frame_tracking_geneus frame_tracking_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frame_tracking_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frame_tracking
)
_generate_msg_lisp(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg"
  "${MSG_I_FLAGS}"
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frame_tracking
)

### Generating Services

### Generating Module File
_generate_module_lisp(frame_tracking
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frame_tracking
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(frame_tracking_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(frame_tracking_generate_messages frame_tracking_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_lisp _frame_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_lisp _frame_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frame_tracking_genlisp)
add_dependencies(frame_tracking_genlisp frame_tracking_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frame_tracking_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frame_tracking
)
_generate_msg_nodejs(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg"
  "${MSG_I_FLAGS}"
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frame_tracking
)

### Generating Services

### Generating Module File
_generate_module_nodejs(frame_tracking
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frame_tracking
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(frame_tracking_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(frame_tracking_generate_messages frame_tracking_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_nodejs _frame_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_nodejs _frame_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frame_tracking_gennodejs)
add_dependencies(frame_tracking_gennodejs frame_tracking_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frame_tracking_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frame_tracking
)
_generate_msg_py(frame_tracking
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg"
  "${MSG_I_FLAGS}"
  "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frame_tracking
)

### Generating Services

### Generating Module File
_generate_module_py(frame_tracking
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frame_tracking
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(frame_tracking_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(frame_tracking_generate_messages frame_tracking_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformation.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_py _frame_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/milab/catkin_ws/src/frame_tracking/msg/pointInformationarray.msg" NAME_WE)
add_dependencies(frame_tracking_generate_messages_py _frame_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(frame_tracking_genpy)
add_dependencies(frame_tracking_genpy frame_tracking_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS frame_tracking_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frame_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/frame_tracking
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(frame_tracking_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(frame_tracking_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(frame_tracking_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET frame_tracking_generate_messages_cpp)
  add_dependencies(frame_tracking_generate_messages_cpp frame_tracking_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frame_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/frame_tracking
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(frame_tracking_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(frame_tracking_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(frame_tracking_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET frame_tracking_generate_messages_eus)
  add_dependencies(frame_tracking_generate_messages_eus frame_tracking_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frame_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/frame_tracking
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(frame_tracking_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(frame_tracking_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(frame_tracking_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET frame_tracking_generate_messages_lisp)
  add_dependencies(frame_tracking_generate_messages_lisp frame_tracking_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frame_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/frame_tracking
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(frame_tracking_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(frame_tracking_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(frame_tracking_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET frame_tracking_generate_messages_nodejs)
  add_dependencies(frame_tracking_generate_messages_nodejs frame_tracking_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frame_tracking)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frame_tracking\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/frame_tracking
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(frame_tracking_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(frame_tracking_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(frame_tracking_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET frame_tracking_generate_messages_py)
  add_dependencies(frame_tracking_generate_messages_py frame_tracking_generate_messages_py)
endif()
