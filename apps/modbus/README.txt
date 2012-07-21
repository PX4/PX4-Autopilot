apps/modbus README
==================

This directory contains a port of last open source version of freemodbus
(BSD license).  The code in this directory is a subset of freemodbus version
1.5.0 (June 6, 2010) that can be downloaded in its entirety from http://developer.berlios.de/project/showfiles.php?group_id=6120.

Directory Structure/Relation to freemodbus-v1.5.0
-------------------------------------------------

The original freemodbus download consists of several directories.  This
subset takes only the contents of one directory, modbus/, that implements
the core modbus logic and integrates that directory into the NuttX build
system.  The mapping between freemodbus-v1.5.0 and the nuttx directories
is shown below:

  --------------------------- ----------------------------------------------
  freemodbus-v1.5.0           Nuttx
  --------------------------- ----------------------------------------------
  All top level .txt files    Not included
  demo/                       Not included.  This directory contains demo
                              and porting code for a variety of platforms.
                              The NuttX demo was ported from the the LINUX
                              demo in this director and can be found at
                              apps/examples/modbus.
  doc/                        Note included.  This directory contains Doxygen
                              support files.
  modbus/                     Included in its entirety in various locations:
         ascii                  apps/modbus/ascii
         functions              apps/modbus/functions
         include                apps/include/modbus
         mb.c                   apps/modbus/mb.c
         rtu                    apps/modbus/rtu
         tcp                    apps/modbus/tcp
  tools/                      Note included.  This directory contain Doxygen
                              tools.
  --------------------------- ----------------------------------------------

So this directory is equivalent to the freemodbus-v1.5.0/modbus
directory except that (1) it may include modifications for the integration
with NuttX and (2) the modbus/include directory was moved to apps/modbus.

The original, unmodified freemodbus-v1.5.0 was checked in as SVN revision
4937

The other directory here, nuttx/, implements the NuttX modbus interface.
It derives from the freemodbus-v1.5.0/demo/LINUX/port directory.

Note
====

The developer of freemodbus, Christian Walter, is still developing Modbus
libraries, although they are now commercial.  See
http://www.embedded-solutions.at/ for further information.
