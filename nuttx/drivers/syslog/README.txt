drivers/syslog README File
==========================

This README file discusses the SYLOG drivers that can be found in the
drivers/syslog directory.  In NuttX, syslog output is equivalent to
debug output and, therefore, the syslogging interfaces are defined in the
header file include/debug.h.

By default, all system log output goes to console (/dev/console).  But that
behavior can be changed by the drivers in this directory.

ramlog.c
--------
  The RAM logging driver is a driver that was intended to support debugging
  output (syslogging) when the normal serial output is not available.  For
  example, if you are using a telnet or USB serial console, the debug
  output will get lost.

  The RAM logging  driver is similar to a pipe in that it saves the
  debugging output in a FIFO in RAM.  It differs from a pipe in numerous
  details as needed to support logging.

  This driver is built when CONFIG_RAMLOG is defined in the Nuttx
  configuration.

  Configuration options:
  
    CONFIG_RAMLOG - Enables the RAM logging feature
    CONFIG_RAMLOG_CONSOLE - Use the RAM logging device as a system console.
      If this feature is enabled (along with CONFIG_DEV_CONSOLE), then all
      console output will be re-directed to a circular buffer in RAM.  This
      is useful, for example, if the only console is a Telnet console.  Then
      in that case, console output from non-Telnet threads will go to the
      circular buffer and can be viewed using the NSH 'dmesg' command.
    CONFIG_RAMLOG_SYSLOG - Use the RAM logging device for the syslogging
      interface.  If this feature is enabled (along with CONFIG_SYSLOG),
      then all debug output (only) will be re-directed to the circular
      buffer in RAM.  This RAM log can be view from NSH using the 'dmesg'
      command.
    CONFIG_RAMLOG_NPOLLWAITERS - The number of threads than can be waiting
      for this driver on poll().  Default: 4

   If CONFIG_RAMLOG_CONSOLE or CONFIG_RAMLOG_SYSLOG is selected, then the
   following may also be provided:

   CONFIG_RAMLOG_CONSOLE_BUFSIZE - Size of the console RAM log.  Default: 1024
