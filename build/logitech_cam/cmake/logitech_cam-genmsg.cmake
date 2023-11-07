# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "logitech_cam: 0 messages, 1 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(logitech_cam_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv" NAME_WE)
add_custom_target(_logitech_cam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "logitech_cam" "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(logitech_cam
  "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/logitech_cam
)

### Generating Module File
_generate_module_cpp(logitech_cam
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/logitech_cam
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(logitech_cam_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(logitech_cam_generate_messages logitech_cam_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(logitech_cam_generate_messages_cpp _logitech_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(logitech_cam_gencpp)
add_dependencies(logitech_cam_gencpp logitech_cam_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS logitech_cam_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(logitech_cam
  "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/logitech_cam
)

### Generating Module File
_generate_module_eus(logitech_cam
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/logitech_cam
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(logitech_cam_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(logitech_cam_generate_messages logitech_cam_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(logitech_cam_generate_messages_eus _logitech_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(logitech_cam_geneus)
add_dependencies(logitech_cam_geneus logitech_cam_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS logitech_cam_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(logitech_cam
  "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/logitech_cam
)

### Generating Module File
_generate_module_lisp(logitech_cam
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/logitech_cam
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(logitech_cam_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(logitech_cam_generate_messages logitech_cam_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(logitech_cam_generate_messages_lisp _logitech_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(logitech_cam_genlisp)
add_dependencies(logitech_cam_genlisp logitech_cam_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS logitech_cam_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(logitech_cam
  "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/logitech_cam
)

### Generating Module File
_generate_module_nodejs(logitech_cam
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/logitech_cam
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(logitech_cam_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(logitech_cam_generate_messages logitech_cam_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(logitech_cam_generate_messages_nodejs _logitech_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(logitech_cam_gennodejs)
add_dependencies(logitech_cam_gennodejs logitech_cam_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS logitech_cam_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(logitech_cam
  "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/logitech_cam
)

### Generating Module File
_generate_module_py(logitech_cam
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/logitech_cam
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(logitech_cam_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(logitech_cam_generate_messages logitech_cam_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa23/class/ee106a-adw/ros_workspace/ee106a_finalproject/src/logitech_cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(logitech_cam_generate_messages_py _logitech_cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(logitech_cam_genpy)
add_dependencies(logitech_cam_genpy logitech_cam_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS logitech_cam_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/logitech_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/logitech_cam
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(logitech_cam_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/logitech_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/logitech_cam
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(logitech_cam_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/logitech_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/logitech_cam
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(logitech_cam_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/logitech_cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/logitech_cam
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(logitech_cam_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/logitech_cam)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/logitech_cam\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/logitech_cam
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(logitech_cam_generate_messages_py sensor_msgs_generate_messages_py)
endif()
