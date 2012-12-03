xtrs README
^^^^^^^^^^^^^

Current status
^^^^^^^^^^^^^^

The xtrs port is not operational yet; some work still needs to be done.

Contents
^^^^^^^^

  o Getting a TRS80 emulator and DOS disks
  o Loading an executable into xtrs
  o Configuring NuttX
  o SDCC
  o Building the SDCC toolchain
  o SDCC Update
  o Newer SDCC Versions

Getting a TRS80 emulator and DOS disks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This port uses a vintage computer based on the Z80, the TRS80.
There's a main page describing the different models of TRS80.
See: http://www.trs-80.com

An emulator for this computer is available to run TRS80 programs on a 
linux platform (http://www.tim-mann.org/xtrs.html).

Other emulators are available for other platforms. 
See http://www.trs-80.com, click on the link Emulators.

TRSDOS, LDOS and other softwares are available at:
http://discover-net.net/~dmkeil/trs80/software/trs-dos.htm

Or you can get TRSDOS 1.3 and 6.1 from this site; it's included with the emulator.
http://discover-net.net/~dmkeil/trs80/model4.htm

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using version 2.7.0 of the SDCC toolchain.

Loading an executable into xtrs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

At http://www.trs-80.com click on the link: Getting a Software Onto an Emulator.

Configuring NuttX
^^^^^^^^^^^^^^^^^

  ostest

    This configuration performs a simple, minimal OS test using
    examples/ostest.  This can be configurated as follows:

       cd tools
       ./configure.sh xtrs/ostest
       cd -
       . ./setenv.sh

  nsh

    This configuration file builds NSH (examples/nsh).  This
    configuration is not functional due to issues with use of the
    simulated serial driver (see the TODO list).

    This configuration can be selected by:

       cd tools
       ./configure.sh xtrs/nsh
       cd -
       . ./setenv.sh

  pashello

    Configures to use examples/pashello for execution from FLASH
    See examples/README.txt for information about pashello.

    This configuration is not usable because the resulting binary
    is too large for the z80 address space.

    This configuration can be selected by:

       cd tools
       ./configure.sh xtrs/pashello
       cd -
       . ./setenv.sh

SDCC
^^^^

These z80 configurations all use the SDCC toolchain (http://sdcc.sourceforge.net/).
Source and pre-built SDCC binaries can be downloaded from the SDCC SourceForge
site: http://sourceforge.net/projects/sdcc/files/ .  Pre-built binaries are
available for Linux, MAC OSX, and for Win32.  Various SDCC options can be
selected with:

  CONFIG_SDCC_POSIX=y   : SDCC for Linux, MAC OSX or Cygwin (see below)
  CONFIG_SDCC_WINDOWS=y : SDCC for Win32

Building the SDCC toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^

You may also want to build your own SDCC toolchain.  You might want to do this,
for example, if you are running under Cygwin and want a Cygwin compatible
SDCC toolchain.

The SDCC toolchain is built with the standard configure/make/make install
sequence.  However, some special actions are required to generate libraries
compatible with this build.  First start with the usual steps

  download
  unpack
  cd sdcc
  ./configure

But before making, we need to apply a patch to the SDCC 2.6.0 source
so that the z80 assembler can handle long symbol names

  Apply sdcc-2.6.0-asz80-symlen.patch
  cd sdcc/device/lib

Then make the SDCC binaries

  cd sdcc
  make

and install SDCC:

  sudo make install

SDCC Update
^^^^^^^^^^^

I have had some problems building sdcc-2.6.0 on my current UBUNTU
release (9.10).  I had other problems building sdcc-2.9.0 on UBUNTU 9.10.
I suspect that the newer gcc toolchains are causing problems for these
older SDCC releases.

A 20091106 snapshot did build with no major problems on UBUNTU 9.10, but
has some compatibilty problems with the older SDCC compiler.  For one, you
will need to change the Z80 assember name and options in the Make.defs
files as follows:

-AS         = as-z80
+AS         = sdasz80
 
-    @$(AS) $(ASFLAGS) $2 $1
+    $(AS) $(ASFLAGS) $1

For another, I had other problems building with that 20091106 that look
like compiler bugs.  If you are using UBUNTU 9.10, you may have to either
(1) downgrade your GCC compiler to a version 3.x compiler and use one of
the older stable releases, or (2) wait for the next stable SDCC release
after 2.9.0.

See below:  If you wish to continue using the older SDCC toolchain, you
must now also add CONFIG_SDCC_OLD=y to your configuration file.

Newer SDCC Versions
^^^^^^^^^^^^^^^^^^^

This is the text of bug 3468951 reported on the SourceForge website:

"Some obsolete stuff in z80sim port," (submitted by Philipp Klaus Krause):

  The simz80 port needs a few updates to work well with current sdcc versions,
  and has some unecessary stuff:

  * The linker name for Make.defs should be sdldz80
  * The assembler name for Make.defs should be sdasz80
  * _asm and _endasm in z80_io.c and z80_irq.c should be replaced by __asm
    and __endasm
  * The --stack-auto --int-long-reent --float-reent options or Make.defs should
     be removed, as they have no effect on sdcc's z80 port
  * The current assembler AFAIK can handle long symbol names, so the
    sdcc-2.6.0-asz80-symlen.patch is unnecessary, and it and the corresponding
    section from the README can be removed.

These changes *have* been incorporated but only partially verified.  In order
to get a successful compilation, I had to copy stdarg.h out of the SDCC source
(at sdcc/device/include/stdarg.h) to include/nuttx/stdarg.h.

There are also some library related issues when you get to the final build
that I have not looked into yet.

If you want to back out these change and continue to use the older toolchain
in your build, simpy define the following in your configuration file:

  CONFIG_SDCC_OLD=y
