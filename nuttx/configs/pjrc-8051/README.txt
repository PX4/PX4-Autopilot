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

