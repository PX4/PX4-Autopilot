NuttX on the Neuros Technology OSD
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

References:
^^^^^^^^^^^

http://wiki.neurostechnology.com/index.php/Main_Page
http://wiki.neurostechnology.com/index.php/OSD_Developer_Home
http://wiki.neurostechnology.com/index.php/DM320_Platform_development
http://wiki.neurostechnology.com/index.php/The_Neuros_and_Open_Source
...

Status:
^^^^^^^

NOTE:  These instructions are for the Neuros development board (and a
rather old version of NuttX),
http://wiki.neurostechnology.com/index.php/OSD_Developer_Board_v1
and have _not_ been updated for the Neuros OSD 1.0 consumer unit
(or for the current version of NuttX),
http://wiki.neurostechnology.com/index.php/Neuros_OSD_1.0

At present, the system only supports a serial console and timer
interrupts so there is not to much that you can do with it.  But I
would be happy to work with anyone who is interested in using it.

General instructions.

1. Download build-0.1.0.tar.gz and nuttx-0.2.3.tar.gz into the same
   <directory>.  These are the current versions as of this writing
   (but could very well new older release now).

2. Unpack, you should now have <directory>/buildroot and
   <directory>/nuttx-0.2.3

3. Rename nuttx-0.2.3 to nuttx

4. Configure NuttX:

   cd <directory>/nuttx/tools
  ./configure.sh ntosd-dm320

5. Build the toolchain:

  cd <directory>/buildroot
  cp configs/c5471-defconfig .config
  make oldconfig
  make

6. Build Nuttx:

  cd <directory>nuttx
  . ./setenv.sh
  make
  mv nuttx /tftpboot/nuttx.dm320

7. Configure the OSD u-boot:

   Neuros Devboard > set ipaddr yy.yy.yy.yy
   Neuros Devboard > set serverip xx.xx.xx.xx
   Neuros Devboard > ...
   Neuros Devboard > run update-ipstatic

   where yy.yy.yy.yy is the OSD IP address and xx.xx.xx.xx is
   the host PC address.

8. Load and run nuttx from uboot

  tftpboot xx.xx.xx.xx nuttx.dm320
  go 1008000

What will run is an a simple OS test that will verify many of the
features of the OS.  (this is nutts/examples/ostest).

