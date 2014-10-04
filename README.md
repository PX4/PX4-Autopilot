UAVCAN - CAN bus for aerospace and robotics
===========================================

[![Coverity Scan](https://scan.coverity.com/projects/1513/badge.svg)](https://scan.coverity.com/projects/1513)

Reference implementation of the [UAVCAN protocol stack](http://uavcan.org/).

## Documentation

* [UAVCAN specification](http://uavcan.org/UAVCAN_specification)
* [Libuavcan overview](http://uavcan.org/Libuavcan)
* [List of platforms officially supported by libuavcan](http://uavcan.org/List_of_platforms_officially_supported_by_libuavcan)
* [Libuavcan tutorials](http://uavcan.org/Libuavcan_tutorials)

## Library development

Despite the fact that the library itself can be used on virtually any platform that has a standard-compliant C++03 or C++11 compiler, the library development process assumes that the host OS is Linux.

Prerequisites:

* Google test library for C++ - gtest
* C++03 *and* C++11 capable compiler with GCC-like interface (e.g. GCC, Clang)
* CMake 2.8+
* Optional: static analysis tool for C++ - cppcheck

Building the debug version and running the unit tests:
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
```
