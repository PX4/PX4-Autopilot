README
^^^^^^

Contents
^^^^^^^^
  o Overview
    - Description
    - Fake Interrupts
    - Timing Fidelity
  o Debugging
  o Issues
    - 64-bit Issues
    - Compiler differences
    - Stack Size Issues
    - Networking Issues
    - X11 Issues
  o BASIC
  o Configurations

Overview
^^^^^^^^

Description
-----------
This README file describes the contents of the build configurations available
for the NuttX "sim" target.  The sim target is a NuttX port that runs as a
user-space program under Linux or Cygwin.  It is a very "low fidelity" embedded
system simulation: This environment does not support any kind of asynchonous
events -- there are nothing like interrupts in this context.  Therefore, there
can be no pre-empting events.

Fake Interrupts
---------------
In order to get timed behavior, the system timer "interrupt handler" is called
from the sim target's IDLE loop.  The IDLE runs whenever there is no other task
running.  So, for example, if a task calls sleep(), then that task will suspend
wanting for the time to elapse.  If nothing else is available to run, then the
IDLE loop runs and the timer increments, eventually re-awakening the sleeping task.

Context switching is based on logic similar to setjmp() and longjmp().

The sim target is used primarily as a development and test platform for new
RTOS features.  It is also of academic interest.  But it has no real-world
application that I know of.

Timing Fidelity
---------------
NOTE:  In order to facility fast testing, the sim target's IDLE loop, by default,
calls the system "interrupt handler" as fast as possible.  As a result, there
really are no noticeable delays when a task sleeps.  However, the task really does
sleep -- but the time scale is wrong.  If you want behavior that is closer to
normal timing, then you can define CONFIG_SIM_WALLTIME=y in your configuration
file.  This configuration setting will cause the sim target's IDLE loop to delay
on each call so that the system "timer interrupt" is called at a rate approximately
correct for the system timer tick rate.  With this definition in the configuration,
sleep() behavior is more or less normal.

Debugging
^^^^^^^^^
One of the best reasons to use the simulation is that is supports great, Linux-
based debugging.  Here are the steps that I following to use the Linux ddd
graphical front-end to GDB:

1. Modify the top-level configuration file.  Enable debug symbols by defining
   the following.

   cd <NuttX-Directory>
   CONFIG_DEBUG_SYMBOLS=y

2. Re-build:

   cd <NuttX-Directory>
   make clean
   make

3. Then start the debugging:

   ddd nuttx &
   gdb> b user_start
   gdb> r

NOTE:  This above steps work fine on both Linux and Cygwin.  On Cygwin, you
will need to start the Cywin-X server before running ddd.

Issues
^^^^^^

64-Bit Issues
-------------
As mentioned above, context switching is based on logic like setjmp() and
longjmp().  This context switching is available for 32-bit and 64-bit
targets.  You must, however, set the correct target in the configuration
before you build: HOST_X86_64 or HOST_X86 for 62- and 32-bit targets,
respectively.  On a 64-bit machine, you can also force the 32-bit build
with CONFIG_SIM_M32=y.

There are other 64-bit issues as well.  For example, addresses are retained in
32-bit unsigned integer types in a few places.  On a 64-bit machine, the 32-bit
address storage may correcupt 64-bit addressing.  NOTE:  This is really a bug --
addresses should not be retained in uint32_t types but rather in uintptr_t types
to avoid issues just like this.

Compiler differences
--------------------

operator new:

  Problem:     "'operator new' takes size_t ('...') as first parameter"
  Workaround:   Add -fpermissive to the compilation flags

Stack Size Issues
-----------------
When you run the NuttX simulation, it uses stacks allocated by NuttX from the
NuttX heap.  The memory management model is exactly the same in the simulation
as it is real, target system.  This is good because this produces a higher
fidelity simulation.

However, when the simulation calls into Linux/Cygwin libraries, it will still
use these small simulation stacks.  This happens, for example, when you call
into the system to get and put characters to the console window or when you
make x11 calls into the system.  The programming model within those libraries
will assume a Linux/Cygwin environment where the stack size grows dynamically
and not the small, limited stacks of a deeply embedded system.

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

  cd ../apps/builtin    # Go to the builtin apps directory
  vi builtin_list.h     # Edit this file and increase the stack size of the add-on
  rm .built *.o         # This will force the builtin apps logic to rebuild

Networking Issues
-----------------
I never did get networking to work on the sim target.  It tries to use the tap device
(/dev/net/tun) to emulate an Ethernet NIC, but I never got it correctly integrated
with the NuttX networking (I probably should try using raw sockets instead).

Update:  Max Holtzberg reports to me that the tap device actually does work properly,
but not in an NSH configuration because of stdio operations freeze the simulation.

X11 Issues
----------
There is an X11-based framebuffer driver that you can use exercise the NuttX graphics
subsystem on the simulator (see the sim/nx11 configuration below).  This may require a
lot of tinkering to get working, depending upon where your X11 installation stores
libraries and header files and how it names libraries.

For example, on UBuntu 9.09, I had to do the following to get a clean build:

    cd /usr/lib/
    sudo ln -s libXext.so.6.4.0 libXext.so

(I also get a segmentation fault at the conclusion of the NX test -- that will need
to get looked into as well).

The X11 examples builds on Cygwin, but does not run.  The last time I tried it,
XOpenDisplay() aborted the program.  UPDATE:  This was caused by the small stack
size and can be fixed by increasing the size of the NuttX stack that calls into
X11.  See the discussion "Stack Size Issues" above.

BASIC
^^^^^

BASIC
=====
  I have used the sim/nsh configuration to test Michael Haardt's BASIC interpreter
  that you can find at apps/interpreters/bas.

    Bas is an interpreter for the classic dialect of the programming language
    BASIC.  It is pretty compatible to typical BASIC interpreters of the 1980s,
    unlike some other UNIX BASIC interpreters, that implement a different
    syntax, breaking compatibility to existing programs.  Bas offers many ANSI
    BASIC statements for structured programming, such as procedures, local
    variables and various loop types.  Further there are matrix operations,
    automatic LIST indentation and many statements and functions found in
    specific classic dialects.  Line numbers are not required.

  There is also a test suite for the interpreter that can be found at
  apps/examples/bastest.

  Configuration
  -------------
  Below are the recommended configuration changes to use BAS with the
  stm32f4discovery/nsh configuration:

  Dependencies:
    CONFIG_LIBC_EXECFUNCS=y      : exec*() functions are required
    CONFIG_LIBM=y                : Some floating point library is required
    CONFIG_LIBC_FLOATINGPOINT=y  : Floating point printing support is required
    CONFIG_LIBC_TMPDIR="/tmp"    : Writable temporary files needed for some commands

  Enable the BASIC interpreter.  Other default options should be okay:
    CONFIG_INTERPRETERS_BAS=y    : Enables the interpreter
    CONFIG_INTERPREPTER_BAS_VT100=y

  The BASIC test suite can be included:
     CONFIG_FS_ROMFS=y           : ROMFS support is needed
     CONFIG_EXAMPLES_BASTEST=y   : Enables the BASIC test setup
     CONFIG_EXAMPLES_BASTEST_DEVMINOR=6
     CONFIG_EXAMPLES_BASTEST_DEVPATH="/dev/ram6"

  Usage
  -----
  This setup will initialize the BASIC test (optional):  This will mount
  a ROMFS file system at /mnt/romfs that contains the BASIC test files:

  nsh> bastest
  Registering romdisk at /dev/ram6
  Mounting ROMFS filesystem at target=/mnt/romfs with source=/dev/ram6
  nsh>

  The interactive interpreter is started like:

  nsh> bas
  bas 2.4
  Copyright 1999-2014 Michael Haardt.
  This is free software with ABSOLUTELY NO WARRANTY.
  >

  Ctrl-D exits the interpreter.

  The test programs can be ran like this:

  nsh> bastest
  Registering romdisk at /dev/ram0
  Mounting ROMFS filesystem at target=/mnt/romfs with source=/dev/ram0
  nsh> bas /mnt/romfs/test01.bas
   1
  hello
   0.0002
   0.0000020
   0.0000002

  nsh>

  Or you can load a test into memory and execute it interactively:

  nsh> bas
  bas 2.4
  Copyright 1999-2014 Michael Haardt.
  This is free software with ABSOLUTELY NO WARRANTY.
  > load "/mnt/romfs/test01.bas"
  > run
   1
  hello
   0.0002
   0.0000020
   0.0000002
  >

Configurations
^^^^^^^^^^^^^^

Common Configuration Information
--------------------------------

  1. Each configuration is maintained in a sub-directory and can be selected
     as follow:

       cd <nuttx-directory>/tools
       ./configure.sh sim/<subdir>
       cd -
       . ./setenv.sh

     If this is a Windows native build, then configure.bat should be used
     instead of configure.sh:

        configure.bat sim\<subdir>

     Where <subdir> is one of the following sub-directories.

  2. All configurations uses the mconf-based configuration tool.  To
     change this configuration using that tool, you should:

     a. Build and install the kconfig mconf tool.  See nuttx/README.txt
        and misc/tools/

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

Configuration Sub-Directories
-----------------------------

configdata

  A unit test for the MTD configuration data driver.

cxxtest


  The C++ standard libary test at apps/examples/cxxtest configuration.  This
  test is used to verify the uClibc++ port to NuttX.

  NOTES
  -----
  1. Before you can use this example, you must first install the uClibc++
     C++ library.  This is located outside of the NuttX source tree at
     misc/uClibc++ in GIT.  See the README.txt file for instructions on
     how to install uClibc++

  2. At present (2012/11/02), exceptions are disabled in this example
     CONFIG_UCLIBCXX_EXCEPTIONS=n).  It is probably not necessary to
     disable exceptions.

  3. Unfortunately, this example will not run now.

     The reason that the example will not run on the simulator has
     to do with when static constructors are enabled:  In the simulator
     it will attempt to execute the static constructors before main()
     starts. BUT... NuttX is not initialized and this results in a crash.

     To really use this example, I will have to think of some way to
     postpone running C++ static initializers until NuttX has been
     initialized.

mount

  Configures to use apps/examples/mount.

mtdpart

  This is the apps/examples/mtdpart test using a MTD RAM driver to
  simulate the FLASH part.

mtdrwb

  This is the apps/examples/mtdrwb test using a MTD RAM driver to
  simulate the FLASH part.

nettest

  Configures to use apps/examples/nettest.  This configuration
  enables networking using the network TAP device.

  NOTES:

  1. The NuttX network is not, however, functional on the Linux TAP
     device yet.

     UPDATE:  The TAP device does apparently work according to a NuttX
     user (provided that it is not used with NSH: NSH waits on readline()
     for console input.  When it calls readline(), the whole system blocks
     waiting from input from the host OS).  My failure to get the TAP
     device working appears to have been a cockpit error.

  2. As of NuttX-5.18, when built on Windows, this test does not try
     to use the TAP device (which is not available on Cygwin anyway),
     but inside will try to use the Cygwin WPCAP library.  Only the
     most preliminary testing has been performed with the Cygwin WPCAP
     library, however.

     NOTE that the IP address is hard-coded in arch/sim/src/up_wpcap.c.
     You will either need to edit your configuration files to use 10.0.0.1
     on the "target" (CONFIG_EXAMPLES_NETTEST_*) or edit up_wpcap.c to
     select the IP address that you want to use.

nsh

  Configures to use the NuttShell at apps/examples/nsh.

  NOTES:

  1. This version has one builtin function:  This configuration:
     apps/examples/hello.

  2. This configuration has BINFS enabled so that the builtin applications
     can be made visible in the file system.  Because of that, the
     build in applications do not work as other examples.

     For example trying to execute the hello builtin application will
     fail:

       nsh> hello
       nsh: hello: command not found
       nsh>

     Unless you first mount the BINFS file system:

       nsh> mount -t binfs /bin
       nsh> ls /bin
       /bin:
         hello
       nsh> echo $PATH
       /bin
       nsh> hello
       Hello, World!!
       nsh>

     Notice that the executable 'hello' is found using the value in the PATH
     variable (which was preset to "/bin").  If the PATH variable were not set
     then you would have to use /bin/hello on the command line.

nsh2

  This is another example that configures to use the NuttShell at apps/examples/nsh.
  Like nsh, this version uses NSH built-in functions:  The nx, nxhello, and
  nxlines examples are included as built-in functions.

  NOTES:

  1. X11 Configuration

     This configuration uses an X11-based framebuffer driver.  Of course, this
     configuration can only be used in environments that support X11!  (And it
     may not even be usable in all of those environments without some "tweaking"
     See discussion below under the nx11 configuration).

nx

  Configures to use apps/examples/nx.

  NOTES:

  1. Special Framebuffer Configuration

     Special simulated framebuffer configuration options:

       CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
       CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
       CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. No Display!

     This version has NO DISPLAY and is only useful for debugging NX
     internals in environments where X11 is not supported.  There is
     and additonal configuration that may be added to include an X11-
     based simulated framebuffer driver:

       CONFIG_SIM_X11FB    - Use X11 window for framebuffer

     See the "nx11" configuration below for more information.

  3. Multi- and Single-User Modes

     The default is the single-user NX implementation.  To select
     the multi-user NX implementation:

       CONFG_NX_MULTIUSER=y
       CONFIG_DISABLE_MQUEUE=n

nx11

  Configures to use apps/examples/nx.  This configuration is similar
  to the nx configuration except that it adds support for an X11-
  based framebuffer driver.  Of course, this configuration can only
  be used in environments that support X11!  (And it may not even
  be usable in all of those environments without some "tweaking").

  1. Special Framebuffer Configuration

     This configuration uses the same special simulated framebuffer
     configuration options as the nx configuration:

       CONFIG_SIM_X11FB    - Use X11 window for framebuffer
       CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
       CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
       CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. X11 Configuration

     But now, since CONFIG_SIM_X11FB is also selected the following
     definitions are needed

       CONFIG_SIM_FBBPP (must match the resolution of the display).
       CONFIG_FB_CMAP=y

     My system has 24-bit color, but packed into 32-bit words so
     the correct setting of CONFIG_SIM_FBBPP is 32.

     For whatever value of CONFIG_SIM_FBBPP is selected, the
     corresponding CONFIG_NX_DISABLE_*BPP setting must not be
     disabled.

  3. Touchscreen Support

     A X11 mouse-based touchscreen simulation can also be enabled
     by setting:

       CONFIG_INPUT=y
       CONFIG_SIM_TOUCHSCREEN=y

     Then you must also have some application logic that will call
     arch_tcinitialize(0) to register the touchscreen driver.  See
     also configuration "touchscreen"

     NOTES:

     a. If you do not have the call to sim_tcinitialize(0), the build
        will mysteriously fail claiming that is can't find up_tcenter()
        and up_tcleave().  That is a consequence of the crazy way that
        the simulation is built and can only be eliminated by calling
        up_simtouchscreen(0) from your application.

     b. You must first up_fbinitialize() before calling up_simtouchscreen()
        or you will get a crash.

     c. Call sim_tcunininitializee() when you are finished with the
        simulated touchscreen.

     d. Enable CONFIG_DEBUG_INPUT=y for touchscreen debug output.

  4. X11 Build Issues

     To get the system to compile under various X11 installations
     you may have to modify a few things.  For example, in order
     to find libXext, I had to make the following change under
     Ubuntu 9.09:

       cd /usr/lib/
       sudo ln -s libXext.so.6.4.0 libXext.so

  5. Multi- and Single-User Modes

     The default is the single-user NX implementation.  To select
     the multi-user NX implementation:

       CONFG_NX_MULTIUSER=y
       CONFIG_DISABLE_MQUEUE=n

   6. apps/examples/nxterm

      This configuration is also set up to use the apps/examples/nxterm
      test instead of apps/examples/nx.  To enable this configuration,
      First, select Multi-User mode as described above.  Then add the
      following definitions to the defconfig file:

       -CONFIG_NXTERM=n
       +CONFIG_NXTERM=y

       -CONFIG_NX_MULTIUSER=n
       +CONFIG_NX_MULTIUSER=y

       -CONFIG_EXAMPLES_NX=y
       +CONFIG_EXAMPLES_NX=n

       -CONFIG_EXAMPLES_NXTERM=n
       +CONFIG_EXAMPLES_NXTERM=y

     See apps/examples/README.txt for further details.

nxffs

  This is the apps/examples/nxffs test using a MTD RAM driver to
  simulate the FLASH part.

nxlines

  This is the apps/examples/nxlines test.

nxwm

  This is a special configuration setup for the NxWM window manager
  UnitTest.  The NxWM window manager can be found here:

    nuttx-code/NxWidgets/nxwm

  The NxWM unit test can be found at:

    nuttx-code/NxWidgets/UnitTests/nxwm

  Documentation for installing the NxWM unit test can be found here:

    nuttx-code/NxWidgets/UnitTests/READEM.txt

  NOTES

  1. There is an issue with running this example under the
     simulation.  In the default configuration, this example will
     run the NxTerm example which waits on readline() for console
     input.  When it calls readline(), the whole system blocks
     waiting from input from the host OS.  So, in order to get
     this example to run, you must comment out the readline call in
     apps/nshlib/nsh_consolemain.c like:

     Index: nsh_consolemain.c
     ===================================================================
     --- nsh_consolemain.c   (revision 4681)
     +++ nsh_consolemain.c   (working copy)
     @@ -117,7 +117,8 @@
        /* Execute the startup script */

      #ifdef CONFIG_NSH_ROMFSETC
     -  (void)nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
     +// REMOVE ME
     +//  (void)nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
      #endif

        /* Then enter the command line parsing loop */
     @@ -130,7 +131,8 @@
            fflush(pstate->cn_outstream);

            /* Get the next line of input */
     -
     +sleep(2); // REMOVE ME
     +#if 0 // REMOVE ME
            ret = readline(pstate->cn_line, CONFIG_NSH_LINELEN,
                           INSTREAM(pstate), OUTSTREAM(pstate));
            if (ret > 0)
     @@ -153,6 +155,7 @@
                        "readline", NSH_ERRNO_OF(-ret));
                nsh_exit(&pstate->cn_vtbl, 1);
              }
     +#endif // REMOVE ME
          }

        /* Clean up */

     UPDATE:  I recently implemented a good UART simulation to driver
     the serial console.  So I do not believe that problem exists and
     I think that the above workaround should no longer be necessary.
     However, I will leave the above text in place until I get then
     oppotunity to verify that the new UART simulation fixes the problem.

ostest

  The "standard" NuttX apps/examples/ostest configuration.

pashello

  Configures to use apps/examples/pashello.

touchscreen

  This configuration uses the simple touchscreen test at
  apps/examples/touchscreen.  This test will create an empty X11 window
  and will print the touchscreen output as it is received from the
  simulated touchscreen driver.

  Since this example uses the simulated frame buffer driver, the
  most of the configuration settings discussed for the "nx11"
  configuration also apply here.  See that discussion above.

  See apps/examples/README.txt for further information about build
  requirements and configuration settings.

traveler

  Configures to build the Traveler first person, 3-D ray casting game at
  apps/graphics/traveler.  This configuration derives fromthe nx11
  configuration and many of the comments there appear here as well.
  This configuration defpends on X11 and, of course, can only be used in
  environments that support X11!  (And it may not even be usable in all of
  those environments without some "tweaking").

  1. Special Framebuffer Configuration

     This configuration uses the same special simulated framebuffer
     configuration options as the nx configuration:

       CONFIG_SIM_X11FB    - Use X11 window for framebuffer
       CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
       CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
       CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. X11 Configuration

     But now, since CONFIG_SIM_X11FB is also selected the following
     definitions are needed

       CONFIG_SIM_FBBPP (must match the resolution of the display).
       CONFIG_FB_CMAP=y

     My system has 24-bit color, but packed into 32-bit words so
     the correct setting of CONFIG_SIM_FBBPP is 32.

  3. X11 Build Issues

     To get the system to compile under various X11 installations
     you may have to modify a few things.  For example, in order
     to find libXext, I had to make the following change under
     Ubuntu 9.09:

       cd /usr/lib/
       sudo ln -s libXext.so.6.4.0 libXext.so

