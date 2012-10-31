misc/uClib++ README
^^^^^^^^^^^^^^^^^^^

This directory contains a version of the uClibc++ C++ library.  This code
originates from http://cxx.uclibc.org/ and has been adapted for NuttX by the
RGMP team (http://rgmp.sourceforge.net/wiki/index.php/Main_Page).

uClibc++ resides in the misc/ directory rather than in the main NuttX source
tree due to licensing issues:  NuttX is licensed under the permissiv
 modified BSD License; uClibc, on the other hand, islicensed under the
 stricter GNU LGPL Version 3 license.

Installation of uClibc++
^^^^^^^^^^^^^^^^^^^^^^^^

If you wish to use uClibc++ with NuttX, you will be required to comply with
the licensing requires of the GNU LGPL Version 3 license.  A simple
installation script is provided at misc/uClibc++/install.sh that can be used
to install the uClibc++ components into the NuttX source tree.

The install script takes only a single arguement: The path to the nuttx
directory.  If your directory structure is the same as the SVN structure
(with misc/ and nuttx/ at the same level), then uClibc++ can be installed
using this command executed from the misc/uClibc++ directory:

  ./install.sh ../../nuttx

If you run the install.sh like this, then it will (1) make sure you
understand that you have tainted the NuttX BSD license with LGPLv3, and (2)
copy the uClibc++ sources files into nuttx/libxx/uClibc++, include/, and
include/cxx.

Building NuttX with uClibc++
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After installing uClibc++ in this way, no additional steps should be
required to build NuttX with the uClibc++ library incorporated:

  $ cd ../../nuttx
  $ . ./setenv.sh
  $ make

Here is how it works:

There is a new file called Make.defs in misc/uClibc/libxx/uClibc++. After
installation, it will reside at nuttx/libxx/uClibc++. The
nuttx/libxx/Makefile will (conditionally) include this Make.defs file:

-include uClibc++/Make.defs

This Make.defs file, if present, will add the uClibc++ source files to the
build, add the uClibc++ subdirectory to the dependency list, and add the
uClibc++ subdirectory to the VPATH. That should, in principle, be all it
takes.

Dependencies
^^^^^^^^^^^^

In order to build libxx (and hence libxx/uClibc++), CONFIG_HAVE_CXX must be
defined in your NuttX configuration file.

The C++ runtime support is provided by GCC libgcc_eh.a and libsupc++.a
libraries.

RGMP
^^^^

The target platform is RGMP X86, it is also updated for TLS support which is
needed by these two libraries. So application can also use TLS on RGMP NuttX
port.

Command to compile and install RGMP:

  $ make
  $ make install
  $ /usr/rgmp/setup
  $ exit

Command to compile and run NUTTX:

  $ cd nuttx/tools
  $ ./configure rgmp/x86/cxxtest
  $ cd ..
  $ make
  $ rgmp_run
