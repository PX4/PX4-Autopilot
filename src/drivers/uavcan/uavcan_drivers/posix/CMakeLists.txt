#
# Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
#

cmake_minimum_required(VERSION 3.9)

project(libuavcan_posix)

#
# Library (header only)
#
install(DIRECTORY include/uavcan_posix DESTINATION include)
