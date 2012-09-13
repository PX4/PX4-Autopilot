arch/x86/include/README.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This directory holds x86-specific header files.  The top-level header files in
arch/x86/include simply include corresponding header files from lower lower-
level chip-specific and architecture-specific directories.

Architecture-Specific Directories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Architecture-specific directories hold common header files for specific x86
architectures.  Separating these header file makes it easy to manage such
things as differences in sizeof(long) on 32- and 64-bit x86 architectures.

i486
  This directory holds definitions appropriate for any instantiation of the
  32-bit i486 architecture.

Chip-Specific directories
^^^^^^^^^^^^^^^^^^^^^^^^^

The same x86 architecture may be realized in different chip implementations.
For SoC chips, in particular, on-chip devices and differing interrupt
structures may require special, chip-specific definitions in these chip-
specific directories.

qemu
  This is the implementation of NuttX on the QEMU x86 simulation.



