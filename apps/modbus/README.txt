apps/modbus README
==================

This directory contains a port of last open source version of FreeModBus
(BSD license).  The code in this directory is a subset of FreeModBus version
1.5.0 (June 6, 2010) that can be downloaded in its entirety from http://developer.berlios.de/project/showfiles.php?group_id=6120.

Directory Structure/Relation to freemodbus-v1.5.0
-------------------------------------------------

The original FreeModBus download consists of several directories.  This
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
4960.

The other directory here, nuttx/, implements the NuttX modbus interface.
It derives from the freemodbus-v1.5.0/demo/LINUX/port directory.

Configuration Options
=====================

In the original freemodbus-v1.5.0 release, the FreeModBus configuration
was controlled by the header file mbconfig.h.  This header file was
eliminated (post revision 4960) and the FreeModBus configuration
was integrated into the NuttX configuration system.

The NuttX-named configuration options that are available include:

    CONFIG_MODBUS - General ModBus support
    CONFIG_MB_ASCII_ENABLED - Modbus ASCII support
    CONFIG_MB_RTU_ENABLED - Modbus RTU support
    CONFIG_MB_TCP_ENABLED - Modbus TCP support
    CONFIG_MB_ASCII_TIMEOUT_SEC - Character timeout value for Modbus ASCII. The
      character timeout value is not fixed for Modbus ASCII and is therefore
      a configuration option. It should be set to the maximum expected delay
      time of the network. Default 1
    CONFIG_MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS - Timeout to wait in ASCII prior
      to enabling transmitter.  If defined the function calls
      vMBPortSerialDelay with the argument CONFIG_MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS
      to allow for a delay before the serial transmitter is enabled. This is
      required because some targets are so fast that there is no time between
      receiving and transmitting the frame. If the master is to slow with
      enabling its receiver then he will not receive the response correctly.
    CONFIG_MB_FUNC_HANDLERS_MAX - Maximum number of Modbus functions codes the
      protocol stack should support. The maximum number of supported Modbus
      functions must be greater than the sum of all enabled functions in this
      file and custom function handlers. If set to small adding more functions
      will fail.
    CONFIG_MB_FUNC_OTHER_REP_SLAVEID_BUF - Number of bytes which should be
      allocated for the Report Slave ID command. This number limits the
      maximum size of the additional segment in the report slave id function.
      See eMBSetSlaveID() for more information on how to set this value. It
      is only used if CONFIG_MB_FUNC_OTHER_REP_SLAVEID_ENABLED is set to 1.
    CONFIG_MB_FUNC_OTHER_REP_SLAVEID_ENABLED - If the Report Slave ID
      function should be enabled.
    CONFIG_MB_FUNC_READ_INPUT_ENABLED - If the Read Input Registers function
      should be enabled.
    CONFIG_MB_FUNC_READ_HOLDING_ENABLED - If the Read Holding Registers
      function should be enabled.
    CONFIG_MB_FUNC_WRITE_HOLDING_ENABLED - If the Write Single Register
      function should be enabled.
    CONFIG_MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED - If the Write Multiple
      registers function should be enabled.
    CONFIG_MB_FUNC_READ_COILS_ENABLED - If the Read Coils function should
      be enabled.
    CONFIG_MB_FUNC_WRITE_COIL_ENABLED - If the Write Coils function should
      be enabled.
    CONFIG_MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED - If the Write Multiple Coils
      function should be enabled.
    CONFIG_MB_FUNC_READ_DISCRETE_INPUTS_ENABLED - If the Read Discrete Inputs
      function should be enabled.
    CONFIG_MB_FUNC_READWRITE_HOLDING_ENABLED - If the Read/Write Multiple
      Registers function should be enabled.

See also other serial settings, in particular:

    CONFIG_SERIAL_TERMIOS - Serial driver supports termios.h interfaces (tcsetattr,
      tcflush, etc.).  If this is not defined, then the terminal settings (baud,
      parity, etc.) are not configurable at runtime; serial streams will not be
      flushed when closed.

Note
====

The developer of FreeModBus, Christian Walter, is still developing Modbus
libraries, although they are now commercial.  See
http://www.embedded-solutions.at/ for further information.
