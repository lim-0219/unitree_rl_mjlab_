# Install script for directory: /home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src

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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/out/libosqp.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osqp" TYPE FILE FILES
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/auxil.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/constants.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/error.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/glob_opts.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/lin_alg.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/osqp.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/osqp_configure.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/proj.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/scaling.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/types.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/util.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/cs.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/polish.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/lin_sys.h"
    "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-src/include/ctrlc.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/out/libosqp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake"
         "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/CMakeFiles/Export/lib/cmake/osqp/osqp-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/CMakeFiles/Export/lib/cmake/osqp/osqp-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/CMakeFiles/Export/lib/cmake/osqp/osqp-targets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/osqp-config.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/src/cmake_install.cmake")
  include("/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/include/cmake_install.cmake")
  include("/home/kwan/unitree_rl_mjlab_/simulate/build_off/_deps/osqp-build/lin_sys/cmake_install.cmake")

endif()

