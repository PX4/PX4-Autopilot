README.txt
^^^^^^^^^^

This directory contains instruction set simulators that were
used to verify NuttX and which are not readily available elsewhere.

These tools were cobbled together using bits and pieces of software
from all over the internet.  Licensing is unknown and you are
certainly at risk if you distribute these in a commercial product.
My recommendation:  For your personal use only.

z80sim
^^^^^^

  This is an emulator for the Z80 instruction set.  It is based
  on the instruction set emulator by Marat Fayzullin but has
  been extended to load Intel hex format files as produced by 
  the SDCC toolchain.

  Sources and licensing
  ^^^^^^^^^^^^^^^^^^^^^

    Marat Fayzullin's Z80 instruction set simulator is available
    here: http://fms.komkon.org/EMUL8/ .  Licensing information
    is available at that site as well.  I am not permitted to
    distribute that package on the internet, but the Makefile
    will automatically download the package.

    Most of the Intel hex logic was copied from some
    http://www.pjrc.com/tech/8051/pm2_docs/intel-hex.html .
    Licensing requires only that I include the authors name
    (Paul Stoffregen) and contact information (paul@ece.orst.edu)
    in the source code.


