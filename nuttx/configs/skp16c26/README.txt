configs/skp16c26/README.txt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. The buildroot package can be used to build an M16C toolchain.  The toolchain
   buildroot can be downloaded from misc/buildroot in the NuttX SVN.  Insructions
   for building the toolchain are provided below.

   However, the target cannot be built because the GNU m16c-elf-ld link fails with 
   the following message:

     m32c-elf-ld: BFD (GNU Binutils) 2.19 assertion fail /home/Owner/projects/nuttx/buildroot/toolchain_build_m32c/binutils-2.19/bfd/elf32-m32c.c:482

   Where the reference line is:

     /* If the symbol is out of range for a 16-bit address,
        we must have allocated a plt entry.  */
     BFD_ASSERT (*plt_offset != (bfd_vma) -1);

   No workaround is known at this time.  This is a show stopper for M16C.

2. A supported version of the M16C toolchain is available here: 

     http://www.kpitgnutools.com/index.php

   This download is free but requires registration.  Unfortunately, this v0901 of
   this toolchain shows the same behavior:

   c:\Hew3\Tools\KPIT Cummins\GNUM16CM32C-ELF\v0901\m32c-elf\bin\m32c-elf-ld.exe: BFD (GNU Binutils) 2.19-GNUM16CM32C_v0901 assertion fail /home/kpit/fsfsrc/binutils-2.19/bfd/elf32-m32c.c:482

It is possible that this error messasge my be telling me -- a very roundabout way --
that I have exceeded the FLASH region, but I think that unlikely (it is difficult
to know if the link does not complete gracefully).

BUILDING THE R8C/M16C/M32C GNU TOOLCHAIN USING BUILDROOT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

NOTE:  See the toolchain issues above -- you may not want to waste your time.

1. CD to the correct directory.

   Change to the directory just above the NuttX installation.  If <nuttx-dir> is
   where NuttX is installed, then cd to <nuttx-dir>/..

2. Get and Install the buildroot Module

   a. Using a release tarball:

     cd <nuttx-dir>/..
     Download the appropriate buildroot package.
     unpack the buildroot package
     rename the directory to buildroot

   b. Using SVN
   
     Check out the misc/buildroot module. SVN checkout instructions:

        svn co https://nuttx.svn.sourceforge.net/svnroot/nuttx nuttx/trunk/misc/buildroot

     Move the buildroot Source Tree and create the archive directory

        mv misc/buildroot .

   Make the archive directory:
  
     mkdir archive

   The <nuttx-dir>/../buildroot is where the toolchain is built;
   The <nuttx-dir>/../archive directory is where toolchain sources will be downloaded.

3. Make sure that NuttX is configured

     cd <nuttx-dir>/tools
     ./configure.sh <nuttx-configuration>
     
4. Configure and Make the buildroot

     cd buildroot
     cp configs/m32c-defconfig-4.2.4 .config
     make oldconfig
     make

   This will download the large source packages for the toolchain and build the toolchain.
   The resulting binaries will be under buildroot/build_m32c.  There will also be a
   large build directory called toolchain_build_m32c; this directory can be removed once
   the build completes successfully.

Cygwin GCC BUILD NOTES
^^^^^^^^^^^^^^^^^^^^^^
   On Cygwin, the buildroot 'make' command will fail with an error like:

   "...
      build/genchecksum cc1-dummy > cc1-checksum.c
      opening cc1-dummy: No such file or directory
   ..."

   This is caused because on Cygwin, host executables will be generated with the extension .exe
   and, apparently, the make variable "exeext" is set incorrectly.  A work around after the
   above occurs is:

      cd toolchain_build_m32c/gcc-4.2.4-initial/gcc	# Go to the directory where error occurred
      mv cc1-dummy.exe cc1-dummy			# Rename the executable without .exe
      rm cc1-checksum.c					# Get rid of the bad generated file

   Then resume the buildroot make:

      cd -						# Back to the buildroot make directory
      make						# Restart the build

   GCC is built twice.  First a initial, "bootstap" GCC is produced in
   toolchain_build_m32c/gcc-4.2.4-initial, then the final GCC is produced in
   toolchain_build_m32c/gcc-4.2.4-final.  The above error will occur twice:  Once for
   the intial GCC build (see above) and once for the final GCC build. For the final GCC
   build, the workaround is the same except that the directory will be
   toolchain_build_m32c/gcc-4.2.4-final/gcc.

   