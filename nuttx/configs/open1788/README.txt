README.txt
==========

This README file discusses the port of NuttX to the WaveShare Open1788 board:
See http://wvshare.com/product/Open1788-Standard.htm. This board features the
NXP LPC1788 MCU

CONTENTS
========

  o Configuration

CONFIGURURATION
===============

  ostest
  ------ 
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Uses the older, OABI, buildroot toolchain.  But that is easily
       reconfigured:

       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain


