NxWidgets/tools README File
===========================

addobjs.sh
----------

  $0 will add all object (.o) files in directory to an archive.

  Usage: tools/addobjs.sh [OPTIONS] <lib-path> <obj-dir>

  Where:
    <lib-path> is the full, absolute path to the library to use
    <obj-dir> is full path to the directory containing the object files to be added
  OPTIONS include:
    -p Prefix to use.  For example, to use arm-elf-ar, add '-p arm-elf-'
    -w Use Windows style paths insted of POSIX paths
    -d Enable script debug
    -h Show this usage information

bitmap_converter.py
-------------------

  This script converts from any image type supported by Python imaging library to
  the RLE-encoded format used by NxWidgets.

indent.sh
---------

  This script uses the Linux 'indent' utility to re-format C source files
  to match the coding style that I use.  It differs from my coding style in that

  - I normally put the traiing */ of a multi-line comment on a separate line,
  - I usually align things vertically (like '='in assignments.

install.sh
----------

  Install a unit test in the NuttX source tree"

  USAGE: tools/install.sh <apps-directory-path> <test-sub-directory>

  Where:
    <apps-directory-path> is the full, absolute path to the NuttX apps/ directory
    <test-sub-directory> is the name of a sub-directory in the UnitTests directory

zipme.sh
--------

  Pack up the NxWidgets tarball for release.

  USAGE:  tools/zipme.sh <version>
