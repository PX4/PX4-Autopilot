#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

cmake_minimum_required(VERSION 2.8)

if(DEFINED CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Debug Release RelWithDebInfo MinSizeRel")
else()
   set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Debug Release RelWithDebInfo MinSizeRel")
endif()

project(uavcan)

#
# DSDL definitions
#
install(DIRECTORY dsdl DESTINATION share/uavcan)

#
# pyuavcan
#
execute_process(COMMAND ./setup.py build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pyuavcan)
install(CODE "execute_process(COMMAND ./setup.py install WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pyuavcan)")

#
# libuavcan
#
add_subdirectory(libuavcan)

#
# libuavcan drivers
#
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    message(STATUS "Adding Linux support library")
    add_subdirectory(libuavcan_drivers/linux)
endif ()