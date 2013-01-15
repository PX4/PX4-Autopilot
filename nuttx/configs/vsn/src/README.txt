
VSN Board Support Package, for the NuttX, Uros Platise <uros.platise@isotel.eu>
===============================================================================
http://www.netClamps.com

The directory contains start-up and board level functions. 
Execution starts in the following order:

 - sysclock, immediately after reset stm32_rcc calls external
   clock configuration when
     CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=y
   is set. It must be set for the VSN board.
   
 - boot, performs initial chip and board initialization
 - sched/os_bringup.c then calls user entry defined in the .config file.


Naming throughout the code
==========================

 - _init(): used to be called once only, after powerup, to perform board
   initialization
 - _start() or called via FS _open(): starts peripheral power, puts it 
   into operation
 - _stop() or called via FS _close(): opposite to _start()
 

System notifications (a sort of run-levels)
===========================================

On the VSN, NSH represents the core application as it supports scripts
easily adaptable for any custom application configuration. NSH is 
invoked as follows (argument runs a script from the /etc/init.d directory):

 - nsh init: on system powerup called by the NuttX APP_START

TODOs: 
 
 - nsh xpowerup: run on external power used to:
   - try to setup an USB serial connection
   - configure SLIP mode, internet
   - start other internet services, such as telnetd, ftpd, httpd
   
 - nsh xpowerdown: run whenever USB recevied suspend signal or
   external power has been removed.
   - used to stop internet services
   
 - nsh batdown: whenever battery is completely discharged

   
Compile notes
===============================

To link-in the sif_main() utility do, in this folder:
 - make context TOPDIR=<path to nuttx top dir>
 
This will result in registering the application into the builtin application
registry.
