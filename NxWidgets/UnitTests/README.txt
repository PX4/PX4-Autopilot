README
======

This directory contains a collection of Unit Tests that can be used to verify
NXWidgets.:

Contents
========
  o Installing and Building the Unit Tests
    1. Setup NuttX
      a) Configure NuttX
      b) Enable C++ Support
      c) Enable Debug Options
      d) Other Possible nuttx/.config changes
      e) Other Possible apps/.config changes
    2. Configure in the Selected Unit Test
  o Work-Arounds
    1. Build Issues
    2. Stack Size Issues with the X11 Simulation
  o Theory of Operation
  o Unit Test Directories
  o Example

Installing and Building the Unit Tests
======================================

1. Setup NuttX

   a) Configure NuttX

   Configure NuttX to run one of the target configurations.  For example,
   let's assume that you are using the sim/nsh2 configuration.  The sim/nsh2
   configuration was specially created for use NXWidgets on the simulation
   platform.  A similar, special configuration stm3210e-eval/nsh2 is also
   for the STM3210E-EVAL available.  However, the unit test can be run on
   other configurations (see steps d and e below).

   NOTE: There are some other special configurationsrecommended for unit-leveling
   testing of NxWM because the configuration is more complex in that case.  These
   are:

   1) sim/nxwmm, or the simulated platform (no touchscreen), and
   2) stm3240g-evel, for the STM3240G-EVAL board (with the STMPE11 touchscreen)

   We will assume the sim/nsh2 configuration in this discussion.  The
   sim/nsh2 configuration is installed as follows:

     cd <nuttx-directory-path>
     make distclean
     cd tools
     ./configure.sh sim/nsh2

   Where:

    <nuttx-directory-path> is the full, absolute path to the NuttX build directory
 
   If you are using the sim/nsh2 or stm3210e-eval configurations, then skip
   to step 2 (Hmmm.. better check 1d) too).

   There may be certain requirements for the configuration that you select...
   for example, certain widget tests may require touchscreen support or special
   font selections.  These test-specific requirements are addressed below under
   "Unit Test Directories"

   b) Enable C++ Support
 
   If you are not using the sim/nsh2 or stm3210e-eval, you will need to add
   the following definitions to the nuttx configuration at nuttx/.config to
   enable C++ support:

     CONFIG_HAVE_CXX=y

   Check first, some configurations already have C++ support enabled (As of this
   writing *ONLY* the sim/nsh2 and stm321-e-eval configurations have C++ support
   pre-enabled).

   d) Enable Debug Options

   If you are running on a simulated target, then you might also want to
   enable debug symbols:

     CONFIG_DEBUG_SYMBOLS=y
 
   Then you can run the simulation using GDB or DDD which is a very powerful
   debugging environment!

   e) Special configuration requirements for the nxwm unit test:
 
     CONFIG_NXCONSOLE=y
     CONFIG_NX_MULTIUSER=y

   f) Other nuttx/.config changes -- NSH configurations only.
 
   If the configuration that you are using supports NSH and NSH built-in tasks
   then all is well.  If it is an NSH configuration, then you will have to define
   the following in your nuttx/.config file as well (if it is not already defined):

     CONFIG_NSH_BUILTIN_APPS=y

   sim/nsh2 and stm3210e-eval/nsh2 already has this setting.  You do not need
   to change anything further in the nuttx/.config file if you are using either
   of these configurations.

   g) Other apps/.config changes -- NON-NSH configurations only.

   Entry Point.  You will need to set the entry point in the .config file.
   For NSH configurations, the entry point will always be "nsh_main" and you
   will see that setting like:

     CONFIG_USER_ENTRYPOINT="nsh_main"

   If you are not using in NSH, then each unit test has a unique entry point.
   That entry point is the name of the unit test directory in all lower case
   plus the suffix "_main".  So, for example, the correct entry for the
   UnitTests/CButton would be:

     CONFIG_USER_ENTRYPOINT="cbutton_main"

   And the correct entry point for UnitTests/nxwm would be:

     CONFIG_USER_ENTRYPOINT="nxwm_main"

   etc.

   For non-NSH configurations (such as the sim/touchscreen) you will have to
   remove the CONFIGURED_APPS seting that contains the user_start function so
   that you use the user_start in the unit test code instead.  So, for example,
   with the sim/touchscreen configuration you need to remove the following from
   the apps/.config file:

     CONFIGURED_APPS += examples/<example> ## REMOVE

   The following step will then install the new, correct directory containing
   the user_start function for the selected unit test.  If you see that NSH
   is configured:
   
     CONFIGURED_APPS += examples/nsh ## DO NOT REMOVE

   Then DO NOT REMOVE the CONFIGURED_APPS setting.  Go back and re-read c)
   above.  Do either c) or d).  Don't do both!

   sim/nsh2 and stm3210e-eval/nsh2 both NSH configurations.  You do not need
   to change anything further in the apps/.config file for any NSH configuration.

2. Configure/Install the Selected Unit Test

   Then reconfigure that to use the Unit Test of your choice:

     cd <nxwidgets-directory>/tools
     ./install.sh <apps-directory-path> <test-sub-directory>
     
   Where:

    <apps-directory-path> is the full, absolute path to the NuttX apps/ directory
    <test-sub-directory> is the name of a sub-directory in the UnitTests directory

3. Instantiate the Configuration

   Before we can make the NXWidgets Library, we have to instantiate the NuttX
   configuration with the installed unit test:

     cd <nuttx-directory-path>
     . ./setenv.sh
     make context

   This will create auto-generated files and will setup symbolic links needed
   in order to build the NXWidgets Library.

4. Adjust the Stack Size

   If using an simulation configuration (like sim/nsh2) and your unit test
   uses X11 as its display device, then you would have to increase the size
   of unit test stack as described below under "Stack Size Issues with the
   X11 Simulation."

5. Build the NXWidgets Library

     cd <nxwidgets-directory>/libnxwidgets
     make TOPDIR=<nuttx-directory-path>

6. Build the NxWM library.

   The NxWM library (libnxwm.a) is required only for the NxWM unit test at
   NxWidgets/UnitTests/nxwm.  For other unit tests, skip to step 7.

     cd <nxwidgets-directory>/nxwm
     make TOPDIR=<nuttx-directory-path>

7. Build NuttX including the unit test and the NXWidgets library

     cd <nuttx-directory-path>
     . ./setenv.sh
     make

Work-Arounds
============

Build Issues
------------
1. I have seen this error on Cygwin building C++ code:

   LD:  nuttx.rel
   ld: skipping incompatible /home/patacongo/projects/nuttx/nuttx/trunk/nuttx/libxx//liblibxx.a when searching for -llibxx
   ld: cannot find -llibxx

   The problem seems to be caused because gcc build code for 32-bit mode and g++ builds code for 64-bit mode.  Add the -m32 option to the g++ command line seems to fix the problem.  In Make.defs:

   CXXFLAGS = -m32 $(ARCHWARNINGSXX) $(ARCHOPTIMIZATION) \
              $(ARCHCPUFLAGSXX) $(ARCHINCLUDESXX) $(ARCHDEFINES) $(EXTRADEFINES) -pipe

2. Stack Size Issues with the X11 Simulation
 
   When you run the NuttX simulation, it uses stacks allocated by NuttX from the
   NuttX heap.  The memory management model is exactly the same in the simulation
   as it is real, target system.  This is good because this produces a higher
   fidelity simulation.

   However, when the simulation calls into Linux/Cygwin libraries, it will still
   use these small simulation stacks.  This happens, for example, when you call
   into the system to get and put characters to the console window or when you
   make x11 calls into the system.  The programming model within those libraries
   will assume a Linux/Cygwin environment where the stack size grows dynamically

   As a consequence, those system libraries may allocate large data structures
   on the stack and overflow the small NuttX stacks.  X11, in particular,
   requires large stacks.  If you are using X11 in the simulation, make sure
   that you set aside a "lot" of stack for the X11 system calls (maybe 8 or 16Kb).
   The stack size for the thread that begins with user start is controlled
   by the configuration setting CONFIG_USERMAIN_STACKSIZE; you may need to
   increase this value to larger number to survive the X11 system calls.

   If you are running X11 applications as NSH add-on programs, then the stack
   size of the add-on program is controlled in another way.  Here are the
   steps for increasing the stack size in that case:

     cd ../apps/namedapps  # Go to the namedapps directory
     vi namedapps_list.h   # Edit this file and increase the stack size of the add-on
     rm .built *.o         # This will force the namedapps logic to rebuild

Theory Of Operation
===================

The NuttX application Makefile at apps/Makefile has some special hooks for
building "external" applications.  In particular, it will automatically
build in the contents any directory call "external/" that may appear in the
apps/ directory.  There is no external/ directory in the repository.  Rather,
this directory may be provided by the user (possibly as a symbolic link) to
add libraries and applications to the standard build from the repository.

The script at tools/install.sh, then, just does a lot of careful validation and
sanity checking.  Then it installs the UnitTest test sub-directory in the apps/
directory by creating a symbolic link in the apps/ directory call "external".
That symbolic link will refer to the selected UnitTest sub-directory.

UnitTests
=========

The following provide simple unit tests for each of the NXWidgets.  In
addition, these unit tests provide examples for the use of each widget
type.

CButton
  Exercises the CButton widget
  Depends on CLabel

CButtonArray
  Exercises the CButtonArray widget

CCheckBox
  Exercises the CCheckBox widget
  Depends on CLabel and CButton.

CGlyphButton
  Exercises the CGlyphButton widget.
  Depends on CLabel and CButton.

CImage
  Exercises the CImage widget

CLabel
  Exercises the CLabel widget

CProgressBar
  Exercises the CProgressBar widget

CRadioButton
  Exercises the CRadioButton and CRadioButtonGroup widgets.
  Depends on CLabel and CButton

CScrollBarHorizontal
  Exercises the ScrollbarHorizontal
  Depends on CSliderHorizontal and CGlyphButton

CScrollBarVertical
  Exercises the ScrollbarHorizontal
  Depends on CSliderVertical and CGlyphButton

CSliderHorizontal
  Exercises the CSliderHorizontal
  Depends on CSliderHorizontalGrip

CSliderVertical
  Exercises the CSliderVertical
  Depends on CSliderVerticalGrip

CTextBox
  Exercises the CTextBox widget
  Depends on CLabel

nxwm
  Exercises the NxWM window manager.
  Use the special configurations nuttx/configs/sim/nxwm or nuttx/configs/stm3240g-eval/nxwm.

Example
=======

1. Configure sim/nsh2
   Where:  nuttx and nuttx/tools directory

   $ cd tools/
   $ ./configure.sh sim/nsh2
   $ cd -

2. Edit nuttx/.config to enable C++ support

   Do nothing... sim/nsh2 already has C++ support enabled.

   Since this is an NSH configuration, the entry point does not need to be
   changed.

3. Install the CButton C++ application (for example)
   Where: <nxwidgets-directory>/tool

   $ ./install.sh ~/projects/nuttx/nuttx/trunk/apps/ CButton
   /home/patacongo/projects/nuttx/nuttx/trunk/apps//external already exists...
     Removing the old symbolic link.
   Creating symbolic link
     - To /home/patacongo/projects/nuttx/nuttx/trunk/NxWidgets/UnitTests/CButton
     - At /home/patacongo/projects/nuttx/nuttx/trunk/apps//external

4. Instantiate the Configuration
   Where: nuttx directory

   $ cd /home/patacongo/projects/nuttx/nuttx/trunk/nuttx
   $ . ./setenv.sh
   $ make context

   This will create auto-generated files and will setup symbolic links needed
   in order to build the NXWidgets Library.

6. Adjust the Stack Size

   If using an simulation configuration (like sim/nsh2) and your unit test
   uses X11 as its display device, then you would have to increase the size
   of unit test stack as described above under "Stack Size Issues."

7. Build the NXWdigets Library
   Where <nxwidgets-directory>/libnxwidgets

   $ cd /home/patacongo/projects/nuttx/nuttx/trunk/NxWidgets/libnxwidgets
   $ make TOPDIR=/home/patacongo/projects/nuttx/nuttx/trunk/nuttx

8. Build NuttX
   Where: nuttx directory

   $ cd /home/patacongo/projects/nuttx/nuttx/trunk/nuttx
   $ make
   ...

8. Run the "bringup" task:

   a. Start NuttX
      Where: nuttx directory

     ./nuttx

     NuttShell (NSH) NuttX-6.9
     nsh> 

   b. Execute NuttX
      Where: NSH command prompt

      NOTE that when you run NSH as a simulation, the commands are echoed
      twice -- once by Linux/Cygwin and once by NuttX:

       nsh> help
       help
       ...
       Builtin Apps:
         nx
         nxhello
         tc
         bringup
       nsh> bringup
       bringup
       bringup [2:128]

   c. Then do a kludgy thing
      Where: NSH command prompt

      The I/O with the Linux/Cygwin simulation is blocking.  So while NSH is
      waiting for input nothing can run (see configs/sim/README.txt for more info).
      One way to make NSH stop asking for input is to sleep.
 
      nsh> sleep 10
      sleep 10

      NOTE 1:  This is not a problem if CONFIG_SCHED_WAITPID is defined in the
      configuration file.  If CONFIG_SCHED_WAITPID is defined, NSH will wait
      for the unit test to complete and it will not be necessary to sleep.

      NOTE 2: CONFIG_SCHED_WAITPID is defined in the sim/nsh2 configuration
      so you can probably ignore this kludgy thing.

   d. The Unit Test runs

      As soon as NSH sleeps, the unit test will run:

      bringup_main: Saying hello from the dynamically constructed instance
      CHelloWorld::HelloWorld: Hello, World!!
      bringup_main: Saying hello from the instance constructed on the stack
      CHelloWorld::HelloWorld: Hello, World!!
      bringup_main: Saying hello from the statically constructed instance
      CHelloWorld::HelloWorld: Hello, World!!

8. Debugging
   Where: nuttx directory

   In the simulation (only) you can use GDB or, better, the graphic
   front-end DDD to debug the code.  Most embedded targets do not
   have good debugging capabilities.

   In order to debug, you have to have build with CONFIG_DEBUG_SYMBOLS=y.
   This setting is preselected in the sim/nsh2 configuration so that
   you don't have to do anything.
   
   Then under Linux or in a Cygwin X11 window, you can start the graphic
   debugger like:

   ddd nuttx &
