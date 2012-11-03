misc/uClib++ README
^^^^^^^^^^^^^^^^^^^

This directory contains a version of the uClibc++ C++ library.  This code
originates from http://cxx.uclibc.org/ and has been adapted for NuttX by the
RGMP team (http://rgmp.sourceforge.net/wiki/index.php/Main_Page).

uClibc++ resides in the misc/ directory rather than in the main NuttX source
tree due to licensing issues:  NuttX is licensed under the permissive
modified BSD License; uClibc, on the other hand, is licensed under the
stricter GNU LGPL Version 3 license.

Contents:
^^^^^^^^^

  o Installation of uClibc++
  o Dependencies
  o NuttX Configuration File Changes
  o Make.defs File Changes
  o Building NuttX with uClibc++
  o Callbacks
  o RGMP

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
include/uClibc++.

Dependencies
^^^^^^^^^^^^

1. The C++ runtime support is provided by GCC libgcc_eh.a and libsupc++.a
  libraries.
2. NuttX C++ support
3. Math library
4. TLS support is currenly provided only under RGMP

NuttX Configuration File Changes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to build libxx (and hence libxx/uClibc++), C++ support must be
enabled.  The following must be defined in your NuttX configuration file.

  CONFIG_HAVE_CXX=y

There are many ways to provide math library support (see nuttx/README.txt).
If you choose to use the NuttX math library, that is enabled as follows:

  CONFIG_LIBM=y

The math libraries depend on the float.h header file that is normally
provided by your tooltchain.  A dummy (and probably wrong) fload.h file
can be installed by setting:

  CONFIG_ARCH_FLOAT_H=y

Exception support can be enabled with:

  CONFIG_UCLIBCXX_EXCEPTION=y

Make.defs File Changes
^^^^^^^^^^^^^^^^^^^^^^

The new files that appear in nuttx/include/uClibc++ must be include-able
as system header files.  So you will need to add 'isystem $(TOPDIR)/include/uClibc++'
to the ARCHINCLUDESXX definition in the NuttX Make.defs file, perhap like:

  -ARCHINCLUDESXX = -I. -isystem $(TOPDIR)/include -isystem $(TOPDIR)/include/cxx
  +ARCHINCLUDESXX = -I. -isystem $(TOPDIR)/include -isystem $(TOPDIR)/include/cxx -isystem $(TOPDIR)/include/uClibc++

And, of course, you no long need to suppress exceptions or run-time typing:

  -ARCHCPUFLAGSXX = -fno-builtin -fno-exceptions -fno-rtti
  +ARCHCPUFLAGSXX = -fno-builtin

If exceptions are disabled via CONFIG_UCLIBCXX_EXCEPTION=n, then -fno-exceptions
is still required.  This logic would handle both cases:

  ifeq ($(CONFIG_UCLIBCXX_EXCEPTION),y)
    ARCHCPUFLAGSXX = -fno-builtin
  else
    ARCHCPUFLAGSXX = -fno-builtin -fno-exceptions
  endif

To get the required libraries into to the NuttX build, it is necessary to add
them to EXTRA_LIBS and to EXTRA_LIBPATHS.  

  LIBSUPXX = ${shell $(CC) --print-file-name=libsupc++.a}
  EXTRA_LIBPATHS = -L "${shell dirname "$(LIBSUPXX)"}"
  EXTRA_LIBS = -lsupc++

NOTE: This assumes that support for these options has been incorporated into
arch/<architecture>/src/Makefile.  As of this writing that is true only for
the ARM and simulation platforms.

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

Callbacks
^^^^^^^^^

The runtime will call this function if exception handling must be abandoned
for any reason.

  void std::terminate(void) throw();

NOTE:  If exception handling is disabled via CONFIG_UCLIBCXX_EXCEPTION, then
this function is always called when an exception would have been thrown.

By default, std::terminate() just calls abort(), but that can be changed
with std:terminate().  std::set_terminated takes a new handler function
as an argument and returns the old handler:

  typedef CODE void (*terminate_handler)(void);
  terminate_handler std::set_terminate(std::terminate_handler func) throw();

The runtime will call this function if an exception is thrown which violates
the function's %exception specification:

  void std::unexpected(void) throw()

This handler defaults to std::terminate but can be re-directed with:

  typedef CODE void (*unexpected_handler)(void);
  unexpected_handler set_unexpected(unexpected_handler) throw();

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
