Libuavcan platform driver for NuttX SocketCAN
================================================

This document describes the Libuavcan v0 driver for NuttX SocketCAN.
The libuavcan driver for NuttX is a header-only C++11 library that implements a fully functional platform interface
for libuavcan and also adds a few convenient wrappers.
It's built on the following Linux components:

* [SocketCAN](http://en.wikipedia.org/wiki/SocketCAN) -
A generic CAN bus stack for Linux.


## Installation



## Using without installation

It is possible to include Libuavcan into another CMake project as a CMake subproject.
In order to do so, extend your `CMakeLists.txt` with the following lines:

```cmake
# Include the Libuavcan CMake subproject
add_subdirectory(
    libuavcan                                       # Path to the Libuavcan repository, modify accordingly
    ${CMAKE_BINARY_DIR}/libuavcan                   # This path can be changed arbitrarily
)

# Add Libuavcan include directories
include_directories(                                # Or use target_include_directories() instead
    libuavcan/libuavcan/include
    libuavcan/libuavcan/include/dsdlc_generated
    libuavcan_linux/include
)

# Link your targets with Libuavcan
target_link_libraries(
    your_target                                     # This is the name of your target
    uavcan
)
```

## Usage

Documentation for each feature is provided in the Doxygen comments in the header files.

NuttX applications that use libuavcan need to link the following libraries:

* libuavcan
