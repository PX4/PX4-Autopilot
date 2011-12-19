pascal/nuttx/README.txt
^^^^^^^^^^^^^^^^^^^^^^^

This directory contains miscellaneous files needed to install the pascal
runtime logic into the NuttX apps/ ource tree.  After installation, the NuttX
apps/ source tree contain the following files

  pcode
  |-- Makefile
  |-- include
  |   `-- Common header files
  |-- libboff
  |   `-- Pascal object format (POFF) library
  `--insn
      |-- include
      |   `-- model-specific header files
      `-- prun
          `-- model-specific source files

This directory contains:

  INSTALL.sh -- The script that performs the operation.  Usage:

    ./INSTALL.sh [-16|-32] <install-dir>

    If you are using the standard NuttX apps/ package, the correct
    location for the <install-dir> is apps/interpreters.  That is
    where the examples and build logic will expect to find the pcode
    sub-directory.

    Example:

    ./INSTALL.sh -16 $PWD/../../../apps/interpreters

  Makefile -- The NuttX makefile for the runtime logic.  This makefile
    is customized to work in the standard apps/ package.  If you intend
    to use your own custom apps/ directory, then this Makefile may
    require some modifications.

  keywords.h -- A version that adjusts build context for the NuttX
    build environment.


