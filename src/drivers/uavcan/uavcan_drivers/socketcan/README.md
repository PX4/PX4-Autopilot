Libuavcan platform driver for NuttX SocketCAN
================================================

This document describes the Libuavcan v0 driver for NuttX SocketCAN.
The libuavcan driver for NuttX is a C++11 library that implements a fully functional platform interface
for libuavcan and also adds a few convenient wrappers.
It's built on the following Linux components:

* [SocketCAN](http://en.wikipedia.org/wiki/SocketCAN) -
A generic CAN bus stack for Linux.

## Usage

Documentation for each feature is provided in the Doxygen comments in the header files.

NuttX applications that use libuavcan need to link the following libraries:

* libuavcan
