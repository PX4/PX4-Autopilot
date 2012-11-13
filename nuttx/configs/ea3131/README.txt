README
^^^^^^

  This README file discusses the port of NuttX to the Embedded Artists
  EA3131 board.

Contents
^^^^^^^^

  o Development Environment
  o GNU Toolchain Options
  o IDEs
  o NuttX buildroot Toolchain
  o Boot Sequence
  o Image Format
  o Image Download to ISRAM
  o Using OpenOCD and GDB
  o On-Demand Paging
  o ARM/EA3131-specific Configuration Options
  o Configurations

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. Raisonance GNU toolchain, or
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_LPC31_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_LPC31_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_LPC31_DEVKITARM=y      : devkitARM under Windows
    CONFIG_LPC31_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_LPC31_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows), devkitARM, and Raisonance toolchains are
  Windows native toolchains.  The CodeSourcey (for Linux) and NuttX buildroot
  toolchains are Cygwin and/or Linux native toolchains. There are several limitations
  to using a Windows based toolchain in a Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project (There is a simple RIDE project
  in the RIDE subdirectory).

  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/lpc31xx,
     arch/arm/src/common, arch/arm/src/arm, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc31xx/lpc31_vectors.S.  With RIDE, I have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh ea3131/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/arm926t-defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Boot Sequence
^^^^^^^^^^^^^
  LPC313x has on chip bootrom which loads properly formatted images from multiple
  sources into SRAM.  These sources include including SPI Flash, NOR Flash, UART,
  USB, SD Card, and NAND Flash.

  In all configurations, NuttX is loaded directly into ISRAM.  NuttX is linked
  to execute from ISRAM, regardless of the boot source.

Image Format
^^^^^^^^^^^^

  In order to use the bootrom bootloader, a special header must be added to the
  beginning of the binary image that includes information about the binary (things
  like the entry point, the size, and CRC's to verify the image.

  NXP provides a Windows program to append such a header to the binary image.
  However, (1) that program won't run under Linux, and (2) when I try it under
  WinXP, Symantec immediately claims that the program is misbehaving and deletes
  it!

  To work around both of these issues, I have created a small program under
  configs/ea3131/tools to add the header.  This program can be built under
  either Linux or Cygwin (and probably other tool environments as well).  That
  tool can be built as follows:

  - cd configs/ea3131/tools
  - make

  Then, to build the NuttX binary ready to load with the bootloader, just
  following these steps:

  - cd tools/                     # Configure Nuttx
  - ./configure.sh ea3131/ostest  # (using the ostest configuration for this example)
  - cd ..                         # Set up environment
  - . ./setenv.sh                 # (see notes below)
  - make                          # Make NuttX.  This will produce nuttx.bin
  - mklpc.sh                      # Make the bootloader binary (nuttx.lpc)

  NOTES:
  
    1. setenv.sh just sets up pathes to the toolchain and also to
       configs/ea3131/tools where mklpc.sh resides. Use of setenv.sh is optional.
       If you don't use setenv.sh, then just set your PATH variable appropriately or
       use the full path to mklpc.sh in the final step.
    2. You can instruct Symantec to ignore the errors and it will stop quarantining
       the NXP program.
    3. The CRC32 logic in configs/ea3131/tools doesn't seem to work.  As a result,
       the CRC is currently disabled in the header:

       RCS file: /cvsroot/nuttx/nuttx/configs/ea3131/tools/lpchdr.c,v
       retrieving revision 1.2
       diff -r1.2 lpchdr.c
       264c264
       <   g_hdr.imageType       = 0x0000000b;
       ---
       >   g_hdr.imageType       = 0x0000000a;

Image Download to ISRAM
^^^^^^^^^^^^^^^^^^^^^^^

Assuming that you already have the FTDI driver installed*, then here is the
are the steps that I use for loading new code into the EA3131:

- Create the bootloader binary, nuttx.lpc, as described above.
- Connected the EA3131 using the FTDI USB port (not the lpc3131 USB port)
  This will power up the EA3131 and start the bootloader.
- Start a terminal emulator (such as TeraTerm) at 115200 8NI.
- Reset the EA3131 and you should see:
  LPC31xx READY FOR PLAIN IMAGE>
- Send the nuttx.lpc file and you should see:
  Download finished

That will load the NuttX binary into ISRAM and attempt to execute it.

*See the LPC313x documentation if you do not have the FTDI driver installed.

Using OpenOCD and GDB
^^^^^^^^^^^^^^^^^^^^^

  I have been using the Olimex ARM-USB-OCD JTAG debugger with the EA3131
  (http://www.olimex.com).  The OpenOCD configuration file is here:
  tools/armusbocb.cfg.  There is also a script on the tools directory that
  I used to start the OpenOCD daemon on my system called oocd.sh.  That
  script would probably require some modifications to work in another
  environment:
  
    - possibly the value of OPENOCD_PATH
    - If you are working under Linux you will need to change any
      occurances of `cygpath -w blablabla` to just blablabla

  Then you should be able to start the OpenOCD daemon like:

    configs/ea3131/tools/oocd.sh $PWD

  Where it is assumed that you are executing oocd.sh from the top level
  directory where NuttX is installed.

  Once the OpenOCD daemon has been started, you can connect to it via
  GDB using the following GDB command:

   arm-nuttx-elf-gdb
   (gdb) target remote localhost:3333

  And you can load the NuttX ELF file:

   (gdb) symbol-file nuttx
   (gdb) load nuttx

On-Demand Paging
^^^^^^^^^^^^^^^^

  There is a configuration that was used to verify the On-Demand Paging
  feature for the ARM926 (see http://nuttx.sourceforge.net/NuttXDemandPaging.html).
  That configuration is contained in the pgnsh sub-directory.  The pgnsh configuration
  is only a test configuration, and lacks some logic to provide the full On-Demand
  Paging solution (see below).

  Page Table Layout:
  ------------------

  The ARM926 MMU uses a page table in memory.  The page table is divided
  into (1) a level 1 (L1) page table that maps 1Mb memory regions to level 2
  page tables (except in the case of 1Mb sections, of course), and (2) a level
  2 (L2) page table that maps the 1Mb memory regions into individual 64Kb, 4kb,
  or 1kb pages.  The pgnsh configuration uses 1Kb pages:  it positions 48x1Kb
  pages at beginning of SRAM (the "locked" memory region), 16x1Kb pages at
  the end of SRAM for the L1 page table, and 44x1Kb pages just before the
  L1 page table.  That leaves 96x1Kb virtual pages in the middle of SRAM for
  the paged memory region; up to 384x1kb of physical pages may be paged into
  this region.  Physical memory map:
  
    11028000 "locked" text region   48x1Kb
    11034000 "paged" text region    96x1Kb
    1104c000 "data" region          32x1Kb
    11054000 L1 page table          16x1Kb
    -------- --------------------- ------
    11058000                       192x1Kb

  The virtual memory map allows more space for the paged region:

    11028000 "locked" text region   48x1Kb
    11034000 "paged" text region   384x1Kb
    11094000 "data" region          32x1Kb
    1109c000 L1 page table          16x1Kb
    -------- --------------------- ------
    110a0000                       480x1Kb

  The L1 contains a single 1Mb entry to span the entire LPC3131 SRAM memory
  region.  The virtual address for this region is 0x11028000.  The offset into
  the L1 page table is given by:
  
    offset = ((0x11028000 >> 20) << 2) = 0x00000440

  The value at that offset into the L1 page table contains the address of the
  L2 page table (0x11056000) plus some extra bits to specify that that entry
  is valid and and points to a 1Kb L1 page table:
  
    11054440 11056013

  Why is the address 11056000 used for the address of the L2 page table?  Isn't
  that inside of the L1 page table?  Yes, this was done to use the preceious
  SRAM memory more conservatively.  If you look at the LPC313x virtual memory
  map, you can see that no virtual addresses above 0x60100000 are used.  That
  corresponds to L1 page table offset 0x0001800 (physical address 0x11055800).
  The rest of the L1 page table is unused and so we reuse it to hold the L2 page
  table (or course, this could cause some really weird addressing L1 mapping
  issues if bad virtual addresses were used in that region -- oh well).  The
  address 0x11056000 is then the first properly aligned memory that can be used
  in that L2 page table area.

  Only only L2 page table will be used to span the LPC3131 SRAM virtual text
  address region (480x1Kb).  That one entry maps the virtual address range of
  0x11000000 through 0x110ffc00.  Each entry maps a 1Kb page of physical memory:
  
    PAGE      VIRTUAL ADDR L2 OFFSET
    --------- ------------ ---------
    Page 0    0x11000000   0x00000000
    Page 1    0x11000400   0x00000004
    Page 2    0x11000800   0x00000008
    ...
    Page 1023 0x110ffc00   0x00000ffc
  
  The "locked" text region begins at an offset of 0x00028000 into that region.
  The 48 page table entries needed to make this region begin at:
  
    offset = ((0x00028000 >> 10) << 2) = 0x00000280

  Each entry contains the address of a physical page in the "locked" text region
  (plus some extra bits to identify domains, page sizes, access privileges, etc.):

    0x11000280 0x1102800b
    0x11000284 0x1102840b
    0x11000288 0x1102880b
    ...

  The locked region is initially unmapped.  But the data region and page table
  regions must be mapped in a similar manner.  Those 
  
    Data:
       Virtual address  = 0x11094000 Offset = 0x00064000
       Physical address = 0x1104c000
       L2 offset        = ((0x00094000 >> 10) << 2) = 0x00000940

    Page table:
       Virtual address  = 0x1109c000 Offset = 0x0009c000
       Physical address = 0x11054000
       L2 offset        = ((0x0009c000 >> 10) << 2) = 0x000009c0

  Build Sequence:
  ---------------

  This example uses a two-pass build.  The top-level Makefile recognizes the
  configuration option CONFIG_BUILD_2PASS and will execute the Makefile in
  configs/ea3131/locked/Makefile to build the first pass object, locked.r.
  This first pass object contains all of the code that must be in the locked
  text region. The Makefile in arch/arm/src/Makefile then includes this 1st
  pass in build, positioning it as controlled by configs/ea3131/scripts/pg-ld.script.

  Finishing the Example:
  ----------------------

  This example is incomplete in that it does not have any media to reload the
  page text region from:  The file configs/ea3131/src/up_fillpage.c is only
  a stub.  That logic to actually reload the page from some storage medium
  (among other things) would have to be implemented in order to complete this
  example.  At present, the example works correctly up to the point where
  up_fillpage() is first called and then fails in the expected way.

  Here are the detailed list of things that would need to be done in addition
  to finishing th up_fillpage() logic (this assumes that SPI NOR FLASH is the
  media on which the NuttX image is stored):

  1. Develop a NOR FLASH layout can can be used to (1) boot the locked text
     section into memory on a reset, and (2) map a virtual fault address
     to an offset into paged text section in NOR FLASH.
  2. Develop/modify the build logic to build the binaries for this NOR
     flash layout: Can the NuttX image be formed as a single image that
     is larger than the IRAM?  Can we boot from such a large image?  If
     so, then no special build modifications are required.  Or, does the
     locked section have to be smaller with a separate paged text section
     image in FLASH?  In this case, some tool will be needed to break
     the nuttx.bin file into the two pieces.
  3. Develop a mechanism to load the NuttX image into SPI NOR FLASH.  A
     basic procedure is already documented in NXP publications: "LPC313x
     Linux Quick Start Guide, Version 2.0" and "AN10811 Programming SPI
     flash on EA3131 boards, V1 (May 1, 2009)."  That procedure may be
     sufficient, depending on the decisions made in (1) and (2):  
  4. Develop a procedure to boot the locked text image from SPI NOR.
     The references and issues related to this are discussed in (2)
     and (3) above.

  Basic support for paging from SPI NOR FLASH can be enabled by adding:

    CONFIG_PAGING_AT45DB=y

  Or:

    CONFIG_PAGING_M25PX=y

  NOTE:  See the TODO list in the top-level directory:
  
    "arch/arm/src/lpc31xx/lpc31_spi.c may or may not be functional.  It was
     reported to be working, but I was unable to get it working with the
     Atmel at45dbxx serial FLASH driver."

  Alternative:
  ------------

  I have implemented an alternative within configs/ea3131/src/up_fillpage.c
  which is probably only useful for testing.  Here is the usage module
  for this alternative

  1. Place the nuttx.bin file on an SD card.
  2. Insert the SD card prior to booting
  3. In up_fillpage(), use the virtual miss address (minus the virtual
     base address) as an offset into the nuttx.bin file, and read the
     required page from that offset in the nuttx.bin file:

     off_t offset = (off_t)vpage - PG_LOCKED_VBASE;
     off_t pos    = lseek(fd, offset, SEEK_SET);
     if (pos != (off_t)-1)
       {
         int ret = read(fd, vpage, PAGESIZE);
       }

  In this way, the paging implementation can do on-demand paging
  from an image file on the SD card.  Problems/issues with this
  approach probably make it only useful for testing:

  1. You would still have to boot the locked section over serial or
     using a bootloader -- it is not clear how the power up boot
     would occur.  For testing, the nuttx.bin file could be both
     provided on the SD card and loaded over serial. 
  2. If the SD card is not in place, the system will crash.
  3. This means that all of the file system logic and FAT file
     system would have to reside in the locked text region.

  And the show-stopper:

  4. There is no MCI driver for the ea3131, yet!

ARM/EA3131-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_ARM926EJS=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc313x

    CONFIG_ARCH_CHIP_name - For use in C code

       CONFIG_ARCH_CHIP_LPC3131

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=ea3131

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_EA3131

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - For most ARM9 architectures, this describes the
      size of installed DRAM.  For the LPC313X, it is used only to
      deterimine how to map the executable regions.  It is SDRAM size
      only if you are executing out of the external SDRAM; or it could
      be NOR FLASH size, external SRAM size, or internal SRAM size.

    CONFIG_DRAM_START - The start address of installed DRAM (physical)

    CONFIG_DRAM_VSTART - The startaddress of DRAM (virtual)

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_IRQPRIO - The LPC313x supports interrupt prioritization

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_BOOTLOADER - Set if you are using a bootloader.

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_BUTTONS -  Enable support for buttons. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.
    CONFIG_ARCH_DMA - Support DMA initialization
    CONFIG_ARCH_LOWVECTORS - define if vectors reside at address 0x0000:00000
      Undefine if vectors reside at address 0xffff:0000
    CONFIG_ARCH_ROMPGTABLE - A pre-initialized, read-only page table is available.
      If defined, then board-specific logic must also define PGTABLE_BASE_PADDR,
      PGTABLE_BASE_VADDR, and all memory section mapping in a file named
      board_memorymap.h.

  Individual subsystems can be enabled:

    CONFIG_LPC31_MCI, CONFIG_LPC31_SPI, CONFIG_LPC31_UART

  External memory available on the board (see also CONFIG_MM_REGIONS)

    CONFIG_LPC31_EXTSRAM0 - Select if external SRAM0 is present
    CONFIG_LPC31_EXTSRAM0HEAP - Select if external SRAM0 should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTSRAM0SIZE - Size (in bytes) of the installed
      external SRAM0 memory
    CONFIG_LPC31_EXTSRAM1 - Select if external SRAM1 is present
    CONFIG_LPC31_EXTSRAM1HEAP - Select if external SRAM1 should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTSRAM1SIZE - Size (in bytes) of the installed
      external SRAM1 memory
    CONFIG_LPC31_EXTSDRAM - Select if external SDRAM is present
    CONFIG_LPC31_EXTSDRAMHEAP - Select if external SDRAM should be
      configured as part of the NuttX heap.
    CONFIG_LPC31_EXTSDRAMSIZE - Size (in bytes) of the installed
      external SDRAM memory
    CONFIG_LPC31_EXTNAND - Select if external NAND is present
    CONFIG_LPC31_EXTSDRAMSIZE - Size (in bytes) of the installed
      external NAND memory

  LPC313X specific device driver settings

    CONFIG_UART_SERIAL_CONSOLE - selects the UART for the
      console and ttys0
    CONFIG_UART_RXBUFSIZE - Characters are buffered as received.
      This specific the size of the receive buffer
    CONFIG_UART_TXBUFSIZE - Characters are buffered before
      being sent.  This specific the size of the transmit buffer
    CONFIG_UART_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UART_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UART_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UART_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

Each EA3131 configuration is maintained in a sudirectory and can be
selected as follow:

    cd tools
    ./configure.sh ea3131/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  locked
    This is not a configuration.  When on-demand page is enabled
    then we must do a two pass link:  The first pass creates an
    intermediate object that has all of the code that must be
    placed in the locked memory partition.  This is logic that
    must be locked in memory at all times.

    The directory contains the logic necessary to do the platform
    specific first pass link for the EA313x.
 
  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interface.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.

  pgnsh:
    This is the same configuration as nsh, but with On-Demand
    paging enabled.  See http://www.nuttx.org/NuttXDemandPaging.html.
    This configuration is an experiment for the purposes of test
    and debug.  At present, this does not produce functioning, 
    usable system
 
  usbserial:
    This configuration directory exercises the USB serial class
    driver at examples/usbserial.  See examples/README.txt for
    more information.

  usbstorage:
    This configuration directory exercises the USB mass storage
    class driver at examples/usbstorage.  See examples/README.txt for
    more information.

