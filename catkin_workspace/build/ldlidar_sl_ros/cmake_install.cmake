# Install script for directory: /home/gansz/mobile_robot/catkin_workspace/src/ldlidar_sl_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/gansz/mobile_robot/catkin_workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/gansz/mobile_robot/catkin_workspace/build/ldlidar_sl_ros/catkin_generated/installspace/ldlidar_sl_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ldlidar_sl_ros/cmake" TYPE FILE FILES
    "/home/gansz/mobile_robot/catkin_workspace/build/ldlidar_sl_ros/catkin_generated/installspace/ldlidar_sl_rosConfig.cmake"
    "/home/gansz/mobile_robot/catkin_workspace/build/ldlidar_sl_ros/catkin_generated/installspace/ldlidar_sl_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ldlidar_sl_ros" TYPE FILE FILES "/home/gansz/mobile_robot/catkin_workspace/src/ldlidar_sl_ros/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros" TYPE PROGRAM FILES
    "/home/gansz/mobile_robot/catkin_workspace/src/ldlidar_sl_ros/scripts/create_udev_rules.sh"
    "/home/gansz/mobile_robot/catkin_workspace/src/ldlidar_sl_ros/scripts/delete_udev_rules.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros" TYPE EXECUTABLE FILES "/home/gansz/mobile_robot/catkin_workspace/devel/lib/ldlidar_sl_ros/ldlidar_sl_ros_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_node"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros" TYPE EXECUTABLE FILES "/home/gansz/mobile_robot/catkin_workspace/devel/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ldlidar_sl_ros/ldlidar_sl_ros_listen_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ldlidar_sl_ros" TYPE DIRECTORY FILES
    "/home/gansz/mobile_robot/catkin_workspace/src/ldlidar_sl_ros/launch"
    "/home/gansz/mobile_robot/catkin_workspace/src/ldlidar_sl_ros/rules"
    "/home/gansz/mobile_robot/catkin_workspace/src/ldlidar_sl_ros/rviz"
    )
endif()

