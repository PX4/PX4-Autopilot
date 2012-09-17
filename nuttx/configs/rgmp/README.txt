RGMP README File
================

RGMP stands for RTOS and GPOS on Multi-Processor.  RGMP is a project for 
running GPOS and RTOS simultaneously on multi-processor platforms. You can
port your favorite RTOS to RGMP together with an unmodified Linux to form a
hybrid operating system. This makes your application able to use both RTOS
and GPOS features.

See http://rgmp.sourceforge.net/wiki/index.php/Main_Page for further
information about RGMP.

The updated build instructions can be found at:
http://rgmp.sourceforge.net/wiki/index.php/Documentation

Ubuntu Build Instructions
--------------------------
Build requirements:
  * x86 PC:
  	Ubuntu 10.04, 10.10 or 11.04
  * OMAP4430 pandaboard:
	Ubuntu 11.04

Run requirements:
  * multi-processor x86 PC:
	Ubuntu 10.04, 10.10 or 11.04
  * OMAP4430 pandaboard:
	Ubuntu 11.04

1. Download RGMP from the following URL:

   http://rgmp.sourceforge.net/wiki/index.php/Download

   You should choose a right verion of RGMP compatible with this NuttX release.
   Extract the tar file:

   $ tar -xjvf rgmp-<RGMP-version>.tar.bz2

2. Get Linux kernel header:

   $ sudo apt-get install linux-headers-$(uname -r)

3. Build and install RGMP:

   $ cd <rgmp-dir>
   $ ./configure
   $ make
   $ sudo make install
   $ sudo /usr/rgmp/setup
   $ exit

4. Configure NuttX.  For example, for the RGMP x86 NSH configuration, do the
   following:

   $ cd <nuttx-dir>
   $ cd tools
   $ ./configure.sh rgmp/x86/nsh
   $ cd ..

5. Build NuttX. Get the binary image at <nuttx-dir>/kernel.img.

   $ cd <nuttx-dir>
   $ make

6. Run NuttX in RGMP:

   $ cd <nuttx-dir>
   $ su
   $ rgmp_run


Other Linux OS Build Instruction
--------------------------------------
Requirements:
  * multi-processor x86 PC
	running Linux kernel 2.6.32, 2.6.35 or 2.6.38
  * OMAP4430 pandaboard
	running Linux kernel 2.6.38

1. Get your running Linux kernel header under /usr/src/linux-headers-$(uname -r)
   directory.

2. Following the Ubuntu steps begin at 3. 

Note: You can configure the RGMP to find Linux kernel header in a different
      place and install RGMP to a different place. See information printed
      by the following instruction:

      $ cd <rgmp-dir>
      $ ./configure -h
   


