function(download_flycap POINTGREY_LIB_VAR POINTGREY_INCLUDE_DIR_VAR)
  if(NOT UNIX)
    message(FATAL_ERROR "Downloading libflycapture for non-linux systems not supported")
  endif()
  include(cmake/TargetArch.cmake)
  set(POINTGREY_URL_BASE "http://www.ptgrey.com/support/downloads/downloads_admin/Dlds/")
  set(POINTGREY_ARCHIVE_x86_64 "flycapture2-2.6.3.2-amd64-pkg.tgz")
  set(POINTGREY_ARCHIVE_i386 "flycapture2-2.6.3.2-i386-pkg.tgz")
  # set(POINTGREY_ARCHIVE_armv7 "flycapture.2.6.3.2_armhf.tar.gz")

  set(POINTGREY_SO_DEB_x86_64 "flycapture2-2.6.3.2-amd64/libflycapture-2.6.3.2_amd64.deb")
  set(POINTGREY_HEADER_DEB_x86_64 "flycapture2-2.6.3.2-amd64/libflycapture-2.6.3.2_amd64-dev.deb")

  target_architecture(POINTGREY_ARCH)
  if(NOT DEFINED POINTGREY_ARCHIVE_${POINTGREY_ARCH})
    message(FATAL_ERROR "No support at this time for ${POINTGREY_ARCH} architecture.")
  endif()
  set(POINTGREY_ARCHIVE ${POINTGREY_ARCHIVE_${POINTGREY_ARCH}})
  if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/${POINTGREY_ARCHIVE}")
    message(STATUS "Using already-downloaded copy of ${POINTGREY_ARCHIVE}")
  else()
    message(STATUS "From ptgrey.com, downloading ${POINTGREY_ARCHIVE} (~9MB)")
    file(DOWNLOAD "${POINTGREY_URL_BASE}${POINTGREY_ARCHIVE}" 
                  "${CMAKE_CURRENT_BINARY_DIR}/${POINTGREY_ARCHIVE}")
  endif()
  message("xx ${CMAKE_BINARY_DIR}")

  set(POINTGREY_LIB "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libflycapture.so.2")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar -zxvf ${POINTGREY_ARCHIVE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  execute_process(
    COMMAND dpkg --extract ${POINTGREY_HEADER_DEB_${POINTGREY_ARCH}} ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND dpkg --extract ${POINTGREY_SO_DEB_${POINTGREY_ARCH}} ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/usr/lib/libflycapture.so.2.6.3.2"
                                     "${POINTGREY_LIB}"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

  set(${POINTGREY_LIB_VAR} ${POINTGREY_LIB} PARENT_SCOPE)
  set(${POINTGREY_INCLUDE_DIR_VAR} "${CMAKE_CURRENT_BINARY_DIR}/usr/include" PARENT_SCOPE)
endfunction()
