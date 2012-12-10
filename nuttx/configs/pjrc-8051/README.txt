pjrc-8051 README
^^^^^^^^^^^^^^^^

This port uses the PJRC 87C52 development system and the SDCC toolchain.

The PJRC 87C52 development system can be obtained from http://www.pjrc.com/.

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using verison 2.6.0 of the SDDC toolchain.

Building the SDCC toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The SDCC toolchain is built with the standard configure/make/make install
sequence.  However, some special actions are required to generate libraries
compatible with this build.  First start with the usual steps

  download
  unpack
  cd sdcc
  ./configure
  make

But before installing, we need to apply a patch to the SDCC 2.6.0 source.
WARNING:  This patch is specific to the particular combination of CFLAGS
that are used in the compilation.  If you change Make.defs, then you will
likely have to change the patch as well.

  Apply sdcc-2.6.0.patch
  cd sdcc/device/lib
  make model-mcs51-stack-auto

Then

  cd sdcc
  make install

Status
^^^^^^

On December 9, 2012, I made updates so that the 8051 port could use the
newest SDCC toolchain (a pre-3.2.1 at that time).  However, when I attempted
to build the PJRC-8051 configuration, I got type incompatibility errors
from sched/os_bringup.c.  From what I gather by googling, this is a compiler
bug related to the --stack-auto option.

I have not been successful working around those bugs and I believe that
these are 8051-related bugs in the SDCC toolchain.  This needs to be
revisited with a later version of SDCC.

This configuration was last successfully built and tested with a ca.
2.6.0 SDCC release.  Support for that older toolchain was removed in
NuttX revision -r5423.  If you wanted to used those older toolchains,
you would need to revert to a pre-r5423 revision.

