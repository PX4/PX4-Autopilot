UAVCAN stack in C++
===================

[![Coverity Scan](https://scan.coverity.com/projects/1513/badge.svg)](https://scan.coverity.com/projects/1513)
[![Travis CI](https://travis-ci.org/UAVCAN/libuavcan.svg?branch=master)](https://travis-ci.org/UAVCAN/libuavcan)
[![Gitter](https://img.shields.io/badge/gitter-join%20chat-green.svg)](https://gitter.im/UAVCAN/general)

Portable reference implementation of the [UAVCAN protocol stack](http://uavcan.org) in C++ for embedded systems
and Linux.

UAVCAN is a lightweight protocol designed for reliable communication in aerospace and robotic applications via CAN bus.

## Documentation

* [UAVCAN website](http://uavcan.org)
* [UAVCAN discussion group](https://groups.google.com/forum/#!forum/uavcan)
* [Libuavcan overview](http://uavcan.org/Implementations/Libuavcan/)
* [List of platforms officially supported by libuavcan](http://uavcan.org/Implementations/Libuavcan/Platforms/)
* [Libuavcan tutorials](http://uavcan.org/Implementations/Libuavcan/Tutorials/)

## Library usage

### Dependencies

* Python 2.7 or 3.3 or newer

Note that this reporitory includes [Pyuavcan](http://uavcan.org/Implementations/Pyuavcan) as a submodule.
Such inclusion enables the library to be built even if pyuavcan is not installed in the system.

### Cloning the repository

```bash
git clone https://github.com/UAVCAN/libuavcan
cd libuavcan
git submodule update --init
```

If this repository is used as a git submodule in your project, make sure to use `--recursive` when updating it.

### Using in a Linux application

Libuavcan can be built as a static library and installed on the system globally as shown below.

```bash
mkdir build
cd build
cmake .. # Default build type is RelWithDebInfo, which can be overriden if needed.
make -j8
sudo make install
```

The following components will be installed:

* Libuavcan headers and the static library
* Generated DSDL headers
* Libuavcan DSDL compiler (a Python script named `libuavcan_dsdlc`)
* Libuavcan DSDL compiler's support library (a Python package named `libuavcan_dsdl_compiler`)

Note that Pyuavcan (an implementation of UAVCAN in Python) will not be installed.
You will need to install it separately if you intend to use the Libuavcan's DSDL compiler in your applications.

It is also possible to use the library as a submodule rather than installing it system-wide.
Please refer to the example applications supplied with the Linux platform driver for more information.

### Using with an embedded system

For ARM targets, it is recommended to use [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded);
however, any other standard-compliant C++ compiler should also work.

#### With Make

Please refer to the [documentation at the UAVCAN website](http://uavcan.org/Implementations/Libuavcan).

#### With CMake

In order to cross-compile the library with CMake, please follow the below instructions.
You will need to provide a CMake toolchain file, `Toolchain-stm32-cortex-m4.cmake` in this example.
If you're not sure what a toolchain file is or how to prepare one, these instructions are probably not for your
use case; please refer to the section about Make instead.

```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/Toolchain-stm32-cortex-m4.cmake
make -j8
```

## Library development

Despite the fact that the library itself can be used on virtually any platform that has a standard-compliant
C++03 or C++11 compiler, the library development process assumes that the host OS is Linux.

Prerequisites:

* Google test library for C++ - gtest (see [how to install on Debian/Ubuntu](http://stackoverflow.com/questions/13513905/how-to-properly-setup-googletest-on-linux))
* C++03 *and* C++11 capable compiler with GCC-like interface (e.g. GCC, Clang)
* CMake 2.8+
* Optional: static analysis tool for C++ - cppcheck (on Debian/Ubuntu use package `cppcheck`)

Building the debug version and running the unit tests:
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make
```

Test outputs can be found in the build directory under `libuavcan`.
Note that unit tests must be executed in real time, otherwise they may produce false warnings;
this implies that they will likely fail if ran on a virtual machine or on a highly loaded system.

Contributors, please follow the [Zubax C++ Coding Conventions](https://kb.zubax.com/x/84Ah).

### Developing with Eclipse

An Eclipse project can be generated like that:

```bash
cmake ../../libuavcan -G"Eclipse CDT4 - Unix Makefiles" \
                      -DCMAKE_ECLIPSE_VERSION=4.3 \
                      -DCMAKE_BUILD_TYPE=Debug \
                      -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
```

Path `../../libuavcan` in the command above points at the directory where the top-level `CMakeLists.txt` is located;
you may need to adjust this per your environment.
Note that the directory where Eclipse project is generated must not be a descendant of the source directory.

### Submitting a Coverity Scan build

First, [get the Coverity build tool](https://scan.coverity.com/download?tab=cxx). Then build the library with it:

```bash
export PATH=$PATH:<coverity-build-tool-directory>/bin/
mkdir build && cd build
cmake <uavcan-source-directory> -DCMAKE_BUILD_TYPE=Debug
cov-build --dir cov-int make -j8
tar czvf uavcan.tgz cov-int
```

Then upload the resulting archive to Coverity.

Automatic check can be triggered by pushing to the branch `coverity_scan`.
