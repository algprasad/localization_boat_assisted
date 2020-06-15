# Install script for directory: /home/alg/aruco3.1.6/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/alg/aruco3.1.6/build/src/libaruco.so.3.1.6"
    "/home/alg/aruco3.1.6/build/src/libaruco.so.3.1"
    "/home/alg/aruco3.1.6/build/src/libaruco.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1.6"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/opt/ros/kinetic/lib/x86_64-linux-gnu:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "/home/alg/aruco3.1.6/src/aruco_cvversioning.h"
    "/home/alg/aruco3.1.6/src/cameraparameters.h"
    "/home/alg/aruco3.1.6/src/dictionary_based.h"
    "/home/alg/aruco3.1.6/src/ippe.h"
    "/home/alg/aruco3.1.6/src/markerdetector_impl.h"
    "/home/alg/aruco3.1.6/src/markermap.h"
    "/home/alg/aruco3.1.6/src/timers.h"
    "/home/alg/aruco3.1.6/src/aruco_export.h"
    "/home/alg/aruco3.1.6/src/cvdrawingutils.h"
    "/home/alg/aruco3.1.6/src/dictionary.h"
    "/home/alg/aruco3.1.6/src/levmarq.h"
    "/home/alg/aruco3.1.6/src/marker.h"
    "/home/alg/aruco3.1.6/src/picoflann.h"
    "/home/alg/aruco3.1.6/src/aruco.h"
    "/home/alg/aruco3.1.6/src/debug.h"
    "/home/alg/aruco3.1.6/src/markerdetector.h"
    "/home/alg/aruco3.1.6/src/markerlabeler.h"
    "/home/alg/aruco3.1.6/src/posetracker.h"
    "/home/alg/aruco3.1.6/src/fractaldetector.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco/fractallabelers" TYPE FILE FILES
    "/home/alg/aruco3.1.6/src/fractallabelers/fractalposetracker.h"
    "/home/alg/aruco3.1.6/src/fractallabelers/fractalmarkerset.h"
    "/home/alg/aruco3.1.6/src/fractallabelers/fractalmarker.h"
    "/home/alg/aruco3.1.6/src/fractallabelers/fractallabeler.h"
    )
endif()

