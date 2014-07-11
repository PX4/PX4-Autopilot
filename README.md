UAVCAN - CAN bus for UAV
======

[![Coverity Scan](https://scan.coverity.com/projects/1513/badge.svg)](https://scan.coverity.com/projects/1513)

Reference implementation of the [UAVCAN protocol stack](http://uavcan.org/).

## Usage

Please refer to <http://uavcan.org/Implementations>.

## Installation

### Posix

Prerequisites:

* CMake v2.8+
* Python 3.2+ or Python 2.7
* Any C++03 or C++11 (preferred) compiler

Installation:
```bash
mkdir build
cd build
cmake .. # Optionally, set the build type: -DCMAKE_BUILD_TYPE=Release (default is RelWithDebInfo)
make
make install
```

The make command will generate the C++ headers for all standard DSDL types and build the static library for libuavcan.

The make install command will install the following components:

* Libuavcan library.
* All standard DSDL type definitions.
* All generated headers for the standard DSDL types.
* Pyuavcan package (it will be installed for the default Python version).
* Libuavcan DSDL compiler - `libuavcan_dsdlc` (use `--help` to get usage info).
* *Linux only:* Linux drivers for libuavcan (see `libuavcan_drivers/linux/`).

### Microcontrollers

No installation required.

#### Using non-make build system

1. Invoke the libuavcan DSDL compiler `libuavcan_dsdlc` to generate C++ headers, or include the headers directly from the system directories if they are installed on your host system (described above).
2. Add the source files and include directories into your project:
    1. Libuavcan source files.
    2. Libuavcan include directory.
    3. Directory containing the hierarchy of generated headers.
    4. Source files of the libuavcan driver for your platform (refer to the directory `libuavcan_drivers/`).
    5. Include directories of the libuavcan driver for your platform.
3. Build.

#### Using make

It's much easier.

1. Include `libuavcan/include.mk`.
2. Use the variables `LIBUAVCAN_*` to retrieve the list of source files and include directories.
3. Use the variable `LIBUAVCAN_DSDLC` to invoke the DSDL compiler to generate headers (i.e. add a make target for that). Alternatively, invoke the compiler by hand, or include the headers directly from the system directories if they are installed on your host system (described above).
4. Use the make script for your platform driver. Normally it's `libuavcan_drivers/<PLATFORM>/driver/include.mk`. Refer to the relevant examples to your platform to see how exactly.

## Library development

Despite the fact that the library itself can be used on virtually any platform that has a standard-compliant C++03 or C++11 compiler, the library development process assumes that the host OS is Linux.

Prerequisites:

* Google test library for C++ - gtest
* Static analysis tool for C++ - cppcheck
* C++03 *and* C++11 capable compiler with GCC-like interface (e.g. GCC, Clang)

Building the debug version, running the unit tests and the static analyzer:
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make            # This may take a lot of time to build multiple versions and run all tests
```
