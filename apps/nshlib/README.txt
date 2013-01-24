apps/nshlib
^^^^^^^^^^^

  This directory contains the NuttShell (NSH) library.  This library can be
  linked with other logic to provide a simple shell application for NuttX.

  - Console/NSH Front End
  - Command Overview
  - Conditional Command Execution
  - Built-In Variables
  - Current Working Directory
    Environment Variables
  - NSH Start-Up Script
  - Simple Commands
  - NSH Configuration Settings
    Command Dependencies on Configuration Settings
    NSH-Specific Configuration Settings
  - Common Problems

Console/NSH Front End
^^^^^^^^^^^^^^^^^^^^^

  Using settings in the configuration file, NSH may be configured to
  use either the serial stdin/out or a telnet connection as the console
  or BOTH.  When NSH is started, you will see the following welcome on
  either console:

    NuttShell (NSH)
    nsh>

  'nsh>' is the NSH prompt and indicates that you may enter a command
   from the console.

Command Overview
^^^^^^^^^^^^^^^^

  This directory contains the NuttShell (NSH).  This is a simple
  shell-like application.  At present, NSH supports the following commands
  forms:

    Simple command:                  <cmd>
    Command with re-directed output: <cmd> > <file>
                                     <cmd> >> <file>
    Background command:              <cmd> &
    Re-directed background command:  <cmd> > <file> &
                                     <cmd> >> <file> &

  Where:

    <cmd>  is any one of the simple commands listed later.
    <file> is the full or relative path to any writable object
           in the filesystem name space (file or character driver).
           Such objects will be referred to simply as files throughout
           this README.

  NSH executes at the mid-priority (128).  Backgrounded commands can
  be made to execute at higher or lower priorities using nice:

    [nice [-d <niceness>>]] <cmd> [> <file>|>> <file>] [&]

  Where <niceness> is any value between -20 and 19 where lower
  (more negative values) correspond to higher priorities.  The
  default niceness is 10.

Conditional Command Execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  An if-then[-else]-fi construct is also supported in order to
  support conditional execution of commands.  This works from the
  command line but is primarily intended for use within NSH scripts
  (see the sh commnd).  The syntax is as follows:

    if <cmd>
    then
      [sequence of <cmd>]
    else
      [sequence of <cmd>]
    fi

Built-In Variables
^^^^^^^^^^^^^^^^^^

  $? - The result of the last simple command execution

Current Working Directory
^^^^^^^^^^^^^^^^^^^^^^^^^

  All path arguments to commands may be either an absolute path or a
  path relative to the current working directory.  The current working
  directory is set using the 'cd' command and can be queried either
  by using the 'pwd' command or by using the 'echo $PWD' command.

  Environment Variables:
  ----------------------

    PWD    - The current working directory
    OLDPWD - The previous working directory

NSH Start-Up Script
^^^^^^^^^^^^^^^^^^^

NSH supports options to provide a start up script for NSH.  In general
this capability is enabled with CONFIG_NSH_ROMFSETC, but has
several other related configuration options as described in the final
section of this README.  This capability also depends on:

  - CONFIG_DISABLE_MOUNTPOINT not set
  - CONFIG_NFILE_DESCRIPTORS > 4
  - CONFIG_FS_ROMFS

Default Start-Up Behavior
-------------------------

The implementation that is provided is intended to provide great flexibility
for the use of Start-Up files.  This paragraph will discuss the general
behavior when all of the configuration options are set to the default
values.

In this default case, enabling CONFIG_NSH_ROMFSETC will cause
NSH to behave as follows at NSH startup time:

- NSH will create a read-only RAM disk (a ROM disk), containing a tiny
  ROMFS filesystem containing the following:

    |--init.d/
         `-- rcS

   Where rcS is the NSH start-up script

- NSH will then mount the ROMFS filesystem at /etc, resulting in:

   |--dev/
   |   `-- ram0
   `--etc/
       `--init.d/
           `-- rcS

- By default, the contents of rcS script are:

    # Create a RAMDISK and mount it at XXXRDMOUNTPOUNTXXX

    mkrd -m 1 -s 512 1024
    mkfatfs /dev/ram1
    mount -t vfat /dev/ram1 /tmp

- NSH will execute the script at /etc/init.d/rcS at start-up (before the
  first NSH prompt.  After execution of the script, the root FS will look
  like:

   |--dev/
   |   |-- ram0
   |   `-- ram1
   |--etc/
   |   `--init.d/
   |       `-- rcS
   `--tmp/

Modifying the ROMFS Image
-------------------------

The contents of the /etc directory are retained in the file
apps/nshlib/nsh_romfsimg.h (OR, if CONFIG_NSH_ARCHROMFS
is defined, include/arch/board/rcs.template).  In order to modify
the start-up behavior, there are three things to study:

1. Configuration Options.
   The additional CONFIG_NSH_ROMFSETC configuration options
   discussed in the final section of this README.

2. tools/mkromfsimg.sh Script.
   The script tools/mkromfsimg.sh creates nsh_romfsimg.h.
   It is not automatically executed.  If you want to change the
   configuration settings associated with creating and mounting
   the /tmp directory, then it will be necessary to re-generate
   this header file using the mkromfsimg.sh script.

   The behavior of this script depends upon three things:

   - The configuration settings of the installed NuttX configuration.
   - The genromfs tool (available from http://romfs.sourceforge.net).
   - The file apps/nshlib/rcS.template (OR, if
     CONFIG_NSH_ARCHROMFS is defined, include/arch/board/rcs.template)

3. rcS.template.
   The file apps/nshlib/rcS.template contains the general form
   of the rcS file; configured values are plugged into this
   template file to produce the final rcS file.

NOTE:

   apps/nshlib/rcS.template generates the standard, default
   nsh_romfsimg.h file.  If CONFIG_NSH_ARCHROMFS is defined
   in the NuttX configuration file, then a custom, board-specific
   nsh_romfsimg.h file residing in configs/<board>/include will be
   used.  NOTE when the OS is configured, include/arch/board will
   be linked to configs/<board>/include.

All of the startup-behavior is contained in rcS.template.  The
role of mkromfsimg.sh is to (1) apply the specific configuration
settings to rcS.template to create the final rcS, and (2) to
generate the header file nsh_romfsimg.h containg the ROMFS
file system image.

Simple Commands
^^^^^^^^^^^^^^^

o [ <expression> ]
o test <expression>

   These are two alternative forms of the same command.  They support
   evaluation of a boolean expression which sets $?.  This command
   is used most frequently as the conditional command following the
   'if' in the if-then[-else]-fi construct.

   Expression Syntax:
   ------------------

     expression = simple-expression | !expression |
                  expression -o expression | expression -a expression

     simple-expression = unary-expression | binary-expression

     unary-expression = string-unary | file-unary

     string-unary = -n string | -z string

     file-unary = -b file | -c file | -d file | -e file | -f file |
                  -r file | -s file | -w file

     binary-expression = string-binary | numeric-binary

     string-binary = string = string | string == string | string != string

     numeric-binary = integer -eq integer | integer -ge integer |
                      integer -gt integer | integer -le integer |
                      integer -lt integer | integer -ne integer

o base64dec [-w] [-f] <string or filepath>

o base64dec [-w] [-f] <string or filepath>

o cat <path> [<path> [<path> ...]]

  This command copies and concatentates all of the files at <path>
  to the console (or to another file if the output is redirected).

o cd [<dir-path>|-|~|..]

  Changes the current working directory (PWD).  Also sets the
  previous working directory environment variable (OLDPWD).

  FORMS:
  ------

    'cd <dir-path>' sets the current working directory to <dir-path>.
    'cd -' sets the current working directory to the previous
       working directory ($OLDPWD).  Equivalent to 'cd $OLDPWD'.
    'cd' or 'cd ~' set the current working directory to the 'home'
       directory.  The 'home' directory can be configured by setting
       CONFIG_LIB_HOMEDIR in the configuration file.  The default
       'home' directory is '/'.
    'cd ..' sets the current working directory to the parent directory.

o cp <source-path> <dest-path>

  Copy of the contents of the file at <source-path> to the location
  in the filesystem indicated by <path-path>

o date [-s "MMM DD HH:MM:SS YYYY"]

  Show or set the current date and time.  This command is only supported
  if the platform supported RTC hardware (CONFIG_RTC=y).

  Only one format is used both on display and when setting the date/time:
  MMM DD HH:MM:SS YYYY.  For example,
  
    data -s "Sep 1 11:30:00 2011"

  24-hour time format is assumed.

o dd if=<infile> of=<outfile> [bs=<sectsize>] [count=<sectors>] [skip=<sectors>]

  Copy blocks from <infile> to <outfile>.  <nfile> or <outfile> may
  be the path to a standard file, a character device, or a block device.

  Examples:

    1. Read from character device, write to regular file.  This will
       create a new file of the specified size filled with zero.

    nsh> dd if=/dev/zero of=/tmp/zeros bs=64 count=16
    nsh> ls -l /tmp
    /tmp:
     -rw-rw-rw-    1024 ZEROS

    2. Read from character device, write to block device.  This will
       fill the entire block device with zeros.

    nsh> ls -l /dev
    /dev:
     brw-rw-rw-       0 ram0
     crw-rw-rw-       0 zero
    nsh> dd if=/dev/zero of=/dev/ram0

    3. Read from a block devic, write to a character device.  This
       will read the entire block device and dump the contents in
       the bit bucket.

    nsh> ls -l /dev
    /dev:
     crw-rw-rw-       0 null
     brw-rw-rw-       0 ram0
    nsh> dd if=/dev/ram0 of=/dev/null

o df

  Show the state of each mounted volume.

  Example:

  nsh> mount
   /etc type romfs
    /tmp type vfat
  nsh> df
    Block  Number
    Size   Blocks     Used Available Mounted on
      64        6        6         0 /etc
     512      985        2       983 /tmp
  nsh> 

o echo [<string|$name> [<string|$name>...]]

  Copy the sequence of strings and expanded environment variables to
  console out (or to a file if the output is re-directed).

o exec <hex-address>

  Execute the user logic at address <hex-address>.  NSH will pause
  until the execution unless the user logic is executed in background
  via 'exec <hex-address> &'

o exit

  Exit NSH.  Only useful if you have started some other tasks (perhaps
  using the 'exec' command') and you would like to have NSH out of the
  way.

o free

  Show the current state of the memory allocator.  For example,

  nsh> free
  free
               total       used       free    largest
  Mem:       4194288    1591552    2602736    2601584

  Where:
    total - This is the total size of memory allocated for use
      by malloc in bytes.
    used - This is the total size of memory occupied by
      chunks handed out by malloc.
    free - This is the total size of memory occupied by
      free (not in use) chunks.
    largest - Size of the largest free (not in use) chunk

o get [-b|-n] [-f <local-path>] -h <ip-address> <remote-path>

  Use TFTP to copy the file at <remote-address> from the host whose IP
  address is identified by <ip-address>.  Other options:

  -f <local-path>
     The file will be saved relative to the current working directory
      unless <local-path> is provided.
  -b|-n
      Selects either binary ("octect") or test ("netascii") transfer
      mode.  Default: text.

o help [-v] [<cmd>]

  Presents summary information about NSH commands to console. Options:

  -v
    Show verbose output will full command usage

  <cmd>
    Show full command usage only for this command

o hexdump <file or device>

  Dump data in hexadecimal format from a file or character device.

o ifconfig [nic_name [<ip-address>|dhcp]] [dr|gw|gateway <dr-address>] [netmask <net-mask>] [dns <dns-address>] [hw <hw-mac>]

  Show the current configuration of the network, for example:

    nsh> ifconfig
    eth0    HWaddr 00:18:11:80:10:06
            IPaddr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

  if uIP statistics are enabled (CONFIG_NET_STATISTICS), then
  this command will also show the detailed state of uIP.

o ifdown <nic-name>

  Take down the interface identified by the name <nic-name>.

  Example:

    ifdown eth0

o ifup <nic-name>

  Bring up down the interface identified by the name <nic-name>.

  Example:

    ifup eth0

o kill -<signal> <pid>

  Send the <signal> to the task identified by <pid>.

o losetup [-d <dev-path>] | [[-o <offset>] [-r] <ldev-path> <file-path>]

  Setup or teardown the loop device:

  1. Teardown the setup for the loop device at <dev-path>:

    losetup d <dev-path>

  2. Setup the loop device at <dev-path> to access the file at <file-path>
     as a block device:

    losetup [-o <offset>] [-r] <dev-path> <file-path>

  Example:

    nsh> dd if=/dev/zero of=/tmp/image bs=512 count=512
    nsh> ls -l /tmp
    /tmp:
     -rw-rw-rw-   262144 IMAGE
    nsh> losetup /dev/loop0 /tmp/image
    nsh> ls -l /dev
    /dev:
     brw-rw-rw-       0 loop0
    nsh> mkfatfs /dev/loop0
    nsh> mount -t vfat /dev/loop0 /mnt/example
    nsh> ls -l /mnt
    ls -l /mnt
    /mnt:
     drw-rw-rw-       0 example/
    nsh> echo "This is a test" >/mnt/example/atest.txt
    nsh> ls -l /mnt/example
    /mnt/example:
     -rw-rw-rw-      16 ATEST.TXT
    nsh> cat /mnt/example/atest.txt
    This is a test
    nsh>

o ls [-lRs] <dir-path>

  Show the contents of the directory at <dir-path>.  NOTE:
  <dir-path> must refer to a directory and no other filesystem
  object.

  Options:
  --------

     -R Show the constents of specified directory and all of its
        sub-directories.
     -s Show the size of the files along with the filenames in the
        listing
     -l Show size and mode information along with the filenames
        in the listing.

o md5 [-f] <string or filepath>

o mb <hex-address>[=<hex-value>][ <hex-byte-count>]
o mh <hex-address>[=<hex-value>][ <hex-byte-count>]
o mw <hex-address>[=<hex-value>][ <hex-byte-count>]

  Access memory using byte size access (mb), 16-bit accesses (mh),
  or 32-bit access (mw).  In each case,

    <hex-address>. Specifies the address to be accessed.  The current
      value at that address will always be read and displayed.
    <hex-address>=<hex-value>.  Read the value, then write <hex-value>
      to the location.
    <hex-byte-count>.  Perform the mb, mh, or mw operation on a total
      of <hex-byte-count> bytes, increment the <hex-address> appropriately
      after each access

  Example

    nsh> mh 0 16
      0 = 0x0c1e
      2 = 0x0100
      4 = 0x0c1e
      6 = 0x0110
      8 = 0x0c1e
      a = 0x0120
      c = 0x0c1e
      e = 0x0130
      10 = 0x0c1e
      12 = 0x0140
      14 = 0x0c1e
    nsh>

o mkdir <path>

  Create the directory at <path>.  All components of of <path>
  except the final directory name must exist on a mounted file
  system; the final directory must not.

  Recall that NuttX uses a pseudo filesystem for its root file system.
  The mkdir command can only be used to create directories in volumes
  set up with the mount command; it cannot be used to create directories
  in the pseudo filesystem.

  Example:
  ^^^^^^^^

    nsh> mkdir /mnt/fs/tmp
    nsh> ls -l /mnt/fs
    /mnt/fs:
     drw-rw-rw-       0 TESTDIR/
     drw-rw-rw-       0 TMP/
    nsh>

o mkfatfs <path>

  Format a fat file system on the block device specified by path.
  NSH provides this command to access the mkfatfs() NuttX API.
  This block device must reside in the NuttX pseudo filesystem and
  must have been created by some call to register_blockdriver() (see
  include/nuttx/fs/fs.h).

o mkfifo <path>

  Creates a FIFO character device anywhere in the pseudo file system,
  creating whatever pseudo directories that may be needed to complete
  the full path.  By convention, however, device drivers are place in
  the standard /dev directory. After it is created, the FIFO device
  may be used as any other device driver. NSH provides this command
  to access the mkfifo() NuttX API.

  Example:
  ^^^^^^^^

    nsh> ls -l /dev
    /dev:
     crw-rw-rw-       0 console
     crw-rw-rw-       0 null
     brw-rw-rw-       0 ram0
    nsh> mkfifo /dev/fifo
    nsh> ls -l /dev
    ls -l /dev
    /dev:
     crw-rw-rw-       0 console
     crw-rw-rw-       0 fifo
     crw-rw-rw-       0 null
     brw-rw-rw-       0 ram0
    nsh>

o mkrd [-m <minor>] [-s <sector-size>] <nsectors>

  Create a ramdisk consisting of <nsectors>, each of size
  <sector-size> (or 512 bytes if <sector-size> is not specified.
  The ramdisk will be registered as /dev/ram<n> (if <n> is not
  specified, mkrd will attempt to register the ramdisk as
  /dev/ram0.

  Example:
  ^^^^^^^^

    nsh> ls /dev
    /dev:
     console
     null
     ttyS0
     ttyS1
    nsh> mkrd 1024
    nsh> ls /dev
    /dev:
     console
     null
     ram0
     ttyS0
     ttyS1
    nsh>

  Once the ramdisk has been created, it may be formatted using
  the mkfatfs command and mounted using the mount command.

  Example:
  ^^^^^^^^
    nsh> mkrd 1024
    nsh> mkfatfs /dev/ram0
    nsh> mount -t vfat /dev/ram0 /tmp
    nsh> ls /tmp
    /tmp:
    nsh>

o mount [-t <fstype> <block-device> <dir-path>]

  The mount command performs one of two different operations.  If no
  paramters are provided on the command line after the mount command,
  then the 'mount' command will enumerate all of the current
  mountpoints on the console.

  If the mount parameters are provied on the command after the 'mount'
  command, then the 'mount' command will mount a file system in the
  NuttX pseudo-file system.  'mount' performs a three way association,
  binding:

    File system.  The '-t <fstype>' option identifies the type of
      file system that has been formatted on the <block-device>.  As
      of this writing, vfat is the only supported value for <fstype>

    Block Device.  The <block-device> argument is the full or relative
      path to a block driver inode in the pseudo filesystem.  By convention,
      this is a name under the /dev sub-directory.  This <block-device>
      must have been previously formatted with the same file system
      type as specified by <fstype>

    Mount Point.  The mount point is the location in the pseudo file
      system where the mounted volume will appear.  This mount point
      can only reside in the NuttX pseudo filesystem.  By convention, this
      mount point is a subdirectory under /mnt.  The mount command will
      create whatever pseudo directories that may be needed to complete
      the full path but the full path must not already exist.

  After the volume has been mounted in the NuttX pseudo file
  system, it may be access in the same way as other objects in the
  file system.

  Examples:
  ^^^^^^^^^

    nsh> ls -l /dev
    /dev:
     crw-rw-rw-       0 console
     crw-rw-rw-       0 null
     brw-rw-rw-       0 ram0
    nsh> ls /mnt
    nsh: ls: no such directory: /mnt
    nsh> mount -t vfat /dev/ram0 /mnt/fs
    nsh> ls -l /mnt/fs/testdir
    /mnt/fs/testdir:
     -rw-rw-rw-      15 TESTFILE.TXT
    nsh> echo "This is a test" >/mnt/fs/testdir/example.txt
    nsh> ls -l /mnt/fs/testdir
    /mnt/fs/testdir:
    -rw-rw-rw-      15 TESTFILE.TXT
     -rw-rw-rw-      16 EXAMPLE.TXT
    nsh> cat /mnt/fs/testdir/example.txt
    This is a test
    nsh>

    nsh> mount
      /etc type romfs
      /tmp type vfat
      /mnt/fs type vfat

o mv <old-path> <new-path>

  Rename the file object at <old-path> to <new-path>.  Both paths must
  reside in the same mounted filesystem.

o nfsmount <server-address> <mount-point> <remote-path>

  Mount the remote NFS server directory <remote-path> at <mount-point> on the target machine.
  <server-address> is the IP address of the remote server.

o ps

  Show the currently active threads and tasks.  For example,

    nsh> ps
    PID   PRI SCHD TYPE   NP STATE    NAME
        0   0 FIFO TASK      READY    Idle Task()
        1 128 RR   TASK      RUNNING  init()
        2 128 FIFO TASK      WAITSEM  nsh_telnetmain()
        3 100 RR   PTHREAD   WAITSEM  <pthread>(21)
    nsh>

o ping [-c <count>] [-i <interval>] <ip-address>

  Test the network communication with a remote peer.  Example,

    nsh> 10.0.0.1
    PING 10.0.0.1 56 bytes of data
    56 bytes from 10.0.0.1: icmp_seq=1 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=2 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=3 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=4 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=5 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=6 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=7 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=8 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=9 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=10 time=0 ms
    10 packets transmitted, 10 received, 0% packet loss, time 10190 ms
    nsh>

o put [-b|-n] [-f <remote-path>] -h <ip-address> <local-path>

  Copy the file at <local-address> to the host whose IP address is
  identified by <ip-address>.  Other options:

  -f <remote-path>
     The file will be saved with the same name on the host unless
      unless <local-path> is provided.
  -b|-n
      Selects either binary ("octect") or test ("netascii") transfer
      mode.  Default: text.

o pwd

  Show the current working directory.

    nsh> cd /dev
    nsh> pwd
    /dev
    nsh>

  Same as 'echo $PWD'

    nsh> echo $PWD
    /dev
    nsh>

o rm <file-path>

  Remove the specified <file-path> name from the mounted file system.
  Recall that NuttX uses a pseudo filesystem for its root file system.
  The rm command can only be used to remove (unlink) files in volumes
  set up with the mount command; it cannot be used to remove names from
  the pseudo filesystem.

  Example:
  ^^^^^^^^

    nsh> ls /mnt/fs/testdir
    /mnt/fs/testdir:
     TESTFILE.TXT
     EXAMPLE.TXT
    nsh> rm /mnt/fs/testdir/example.txt
    nsh> ls /mnt/fs/testdir
    /mnt/fs/testdir:
     TESTFILE.TXT
    nsh>

o rmdir <dir-path>

  Remove the specified <dir-path> directory from the mounted file system.
  Recall that NuttX uses a pseudo filesystem for its root file system. The
  rmdir command can only be used to remove directories from volumes set up
  with the mount command; it cannot be used to remove directories from the
  pseudo filesystem. 

  Example:
  ^^^^^^^^

    nsh> mkdir /mnt/fs/tmp
    nsh> ls -l /mnt/fs
    /mnt/fs:
     drw-rw-rw-       0 TESTDIR/
     drw-rw-rw-       0 TMP/
    nsh> rmdir /mnt/fs/tmp
    nsh> ls -l /mnt/fs
    ls -l /mnt/fs
    /mnt/fs:
     drw-rw-rw-       0 TESTDIR/
    nsh>

o set <name> <value>

  Set the environment variable <name> to the sting <value>.
  For example,

    nsh> echo $foobar

    nsh> set foobar foovalue
    nsh> echo $foobar
    foovalue
    nsh>

o sh <script-path>

  Execute the sequence of NSH commands in the file referred
  to by <script-path>.

o sleep <sec>

  Pause execution (sleep) of <sec> seconds.

o unset <name>

  Remove the value associated with the environment variable
  <name>.  Example:

    nsh> echo $foobar
    foovalue
    nsh> unset foobar
    nsh> echo $foobar

    nsh>

 o urldecode [-f] <string or filepath>
 
 o urlencode [-f] <string or filepath>

o usleep <usec>

  Pause execution (sleep) of <usec> microseconds.

o wget [-o <local-path>] <url>

  Use HTTP to copy the file at <url> to the current directory.
  Options:

  -o <local-path>
     The file will be saved relative to the current working directory
     and with the same name as on the HTTP server unless <local-path>
     is provided.

o xd <hex-address> <byte-count>

  Dump <byte-count> bytes of data from address <hex-address>

  Example:
  ^^^^^^^^

    nsh> xd 410e0 512
    Hex dump:
    0000: 00 00 00 00 9c 9d 03 00 00 00 00 01 11 01 10 06 ................
    0010: 12 01 11 01 25 08 13 0b 03 08 1b 08 00 00 02 24 ....%..........$
    ...
    01f0: 08 3a 0b 3b 0b 49 13 00 00 04 13 01 01 13 03 08 .:.;.I..........
    nsh>

NSH Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^

The availability of the above commands depends upon features that
may or may not be enabled in the NuttX configuration file.  The 
following table indicates the dependency of each command on NuttX
configuration settings.  General configuration settings are discussed
in the NuttX Porting Guide.  Configuration settings specific to NSH
as discussed at the bottom of this README file.

Command Dependencies on Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Command    Depends on Configuration
  ---------- --------------------------
  [          !CONFIG_NSH_DISABLESCRIPT
  base64dec  CONFIG_NETUTILS_CODECS && CONFIG_CODECS_BASE64
  base64enc  CONFIG_NETUTILS_CODECS && CONFIG_CODECS_BASE64
  cat        CONFIG_NFILE_DESCRIPTORS > 0
  cd         !CONFIG_DISABLE_ENVIRON && CONFIG_NFILE_DESCRIPTORS > 0
  cp         CONFIG_NFILE_DESCRIPTORS > 0
  dd         CONFIG_NFILE_DESCRIPTORS > 0
  df         !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_READABLE (see note 3)
  echo       --
  exec       --
  exit       --
  free       --
  get        CONFIG_NET && CONFIG_NET_UDP && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NET_BUFSIZE >= 558  (see note 1)
  help       --
  hexdump    CONFIG_NFILE_DESCRIPTORS > 0
  ifconfig   CONFIG_NET
  ifdown     CONFIG_NET
  ifup       CONFIG_NET
  kill       !CONFIG_DISABLE_SIGNALS
  losetup    !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0
  ls         CONFIG_NFILE_DESCRIPTORS > 0
  md5        CONFIG_NETUTILS_CODECS && CONFIG_CODECS_HASH_MD5
  mb,mh,mw   ---
  mkdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_WRITABLE (see note 4)
  mkfatfs    !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_FAT
  mkfifo     CONFIG_NFILE_DESCRIPTORS > 0
  mkrd       !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_WRITABLE (see note 4)
  mount      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_READABLE (see note 3)
  mv         !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_WRITABLE (see note 4)
  nfsmount   !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NET && CONFIG_NFS
  ping       CONFIG_NET && CONFIG_NET_ICMP && CONFIG_NET_ICMP_PING  && !CONFIG_DISABLE_CLOCK && !CONFIG_DISABLE_SIGNALS
  ps         --
  put        CONFIG_NET && CONFIG_NET_UDP && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NET_BUFSIZE >= 558 (see note 1,2)
  pwd        !CONFIG_DISABLE_ENVIRON && CONFIG_NFILE_DESCRIPTORS > 0
  rm         !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_WRITABLE (see note 4)
  rmdir      !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_WRITABLE (see note 4)
  set        !CONFIG_DISABLE_ENVIRON
  sh         CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0 && !CONFIG_NSH_DISABLESCRIPT
  sleep      !CONFIG_DISABLE_SIGNALS
  test       !CONFIG_NSH_DISABLESCRIPT
  umount     !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_FS_READABLE
  unset      !CONFIG_DISABLE_ENVIRON
  urldecode  CONFIG_NETUTILS_CODECS && CONFIG_CODECS_URLCODE
  urlencode  CONFIG_NETUTILS_CODECS && CONFIG_CODECS_URLCODE
  usleep     !CONFIG_DISABLE_SIGNALS
  get        CONFIG_NET && CONFIG_NET_TCP && CONFIG_NFILE_DESCRIPTORS > 0
  xd         ---

* NOTES:
  1. Because of hardware padding, the actual buffersize required for put and get
     operations size may be larger.
  2. Special TFTP server start-up optionss will probably be required to permit
     creation of file for the correct operation of the put command.
  3. CONFIG_FS_READABLE is not a user configuration but is set automatically
     if any readable filesystem is selected.  At present, this is either CONFIG_FS_FAT
     and CONFIG_FS_ROMFS.
  4. CONFIG_FS_WRITABLE is not a user configuration but is set automatically
     if any writable filesystem is selected.  At present, this is only CONFIG_FS_FAT.

In addition, each NSH command can be individually disabled via one of the following
settings.  All of these settings make the configuration of NSH potentially complex but
also allow it to squeeze into very small memory footprints.

  CONFIG_NSH_DISABLE_BASE64DEC, CONFIG_NSH_DISABLE_BASE64ENC, CONFIG_NSH_DISABLE_CAT,
  CONFIG_NSH_DISABLE_CD,        CONFIG_NSH_DISABLE_CP,        CONFIG_NSH_DISABLE_DD,
  CONFIG_NSH_DISABLE_DF,        CONFIG_NSH_DISABLE_ECHO,      CONFIG_NSH_DISABLE_EXEC,
  CONFIG_NSH_DISABLE_EXIT,      CONFIG_NSH_DISABLE_FREE,      CONFIG_NSH_DISABLE_GET,
  CONFIG_NSH_DISABLE_HELP,      CONFIG_NSH_DISABLE_HEXDUMP,   CONFIG_NSH_DISABLE_IFCONFIG,
  CONFIG_NSH_DISABLE_IFUPDOWN,  CONFIG_NSH_DISABLE_KILL,      CONFIG_NSH_DISABLE_LOSETUP,
  CONFIG_NSH_DISABLE_LS,        CONFIG_NSH_DISABLE_MD5        CONFIG_NSH_DISABLE_MB,
  CONFIG_NSH_DISABLE_MKDIR,     CONFIG_NSH_DISABLE_MKFATFS,   CONFIG_NSH_DISABLE_MKFIFO,
  CONFIG_NSH_DISABLE_MKRD,      CONFIG_NSH_DISABLE_MH,        CONFIG_NSH_DISABLE_MOUNT,
  CONFIG_NSH_DISABLE_MW,        CONFIG_NSH_DISABLE_MV,        CONFIG_NSH_DISABLE_NFSMOUNT,
  CONFIG_NSH_DISABLE_PS,        CONFIG_NSH_DISABLE_PING,      CONFIG_NSH_DISABLE_PUT,
  CONFIG_NSH_DISABLE_PWD,       CONFIG_NSH_DISABLE_RM,        CONFIG_NSH_DISABLE_RMDIR,
  CONFIG_NSH_DISABLE_SET,       CONFIG_NSH_DISABLE_SH,        CONFIG_NSH_DISABLE_SLEEP,
  CONFIG_NSH_DISABLE_TEST,      CONFIG_NSH_DISABLE_UMOUNT,    CONFIG_NSH_DISABLE_UNSET,
  CONFIG_NSH_DISABLE_URLDECODE, CONFIG_NSH_DISABLE_URLENCODE, CONFIG_NSH_DISABLE_USLEEP,
  CONFIG_NSH_DISABLE_WGET,      CONFIG_NSH_DISABLE_XD

Verbose help output can be suppressed by defining CONFIG_NSH_HELP_TERSE.  In that
case, the help command is still available but will be slightly smaller.

NSH-Specific Configuration Settings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  The behavior of NSH can be modified with the following settings in
  the configs/<board-name>/defconfig file:

  * CONFIG_NSH_BUILTIN_APPS
      Support external registered, "builtin" applications that can be
      executed from the NSH command line (see apps/README.txt for
      more information).
  
  * CONFIG_NSH_FILEIOSIZE
      Size of a static I/O buffer used for file access (ignored if
      there is no filesystem). Default is 1024.

  * CONFIG_NSH_STRERROR
      strerror(errno) makes more readable output but strerror() is
      very large and will not be used unless this setting is 'y'.
      This setting depends upon the strerror() having been enabled
      with CONFIG_LIBC_STRERROR.

  * CONFIG_NSH_LINELEN
      The maximum length of one command line and of one output line.
      Default: 80

  * CONFIG_NSH_NESTDEPTH
      The maximum number of nested if-then[-else]-fi sequences that
      are permissable.  Default: 3

  * CONFIG_NSH_DISABLESCRIPT
      This can be set to 'y' to suppress support for scripting.  This
      setting disables the 'sh', 'test', and '[' commands and the
      if-then[-else]-fi construct.  This would only be set on systems
      where a minimal footprint is a necessity and scripting is not.

  * CONFIG_NSH_DISABLEBG
      This can be set to 'y' to suppress support for background
      commands.  This setting disables the 'nice' command prefix and
      the '&' command suffix.  This would only be set on systems
      where a minimal footprint is a necessity and background command
      execution is not.

  * CONFIG_NSH_MMCSDMINOR
      If the architecture supports an MMC/SD slot and if the NSH
      architecture specific logic is present, this option will provide
      the MMC/SD minor number, i.e., the MMC/SD block driver will
      be registered as /dev/mmcsdN where N is the minor number.
      Default is zero.

  * CONFIG_NSH_ROMFSETC
      Mount a ROMFS filesystem at /etc and provide a startup script
      at /etc/init.d/rcS.  The default startup script will mount
      a FAT FS RAMDISK at /tmp but the logic is easily extensible.

  * CONFIG_NSH_CONSOLE
      If CONFIG_NSH_CONSOLE is set to 'y', then a serial
      console front-end is selected.

      Normally, the serial console device is a UART and RS-232
      interface.  However, if CONFIG_USBDEV is defined, then a USB
      serial device may, instead, be used if the one of
      the following are defined:

      CONFIG_PL2303 and CONFIG_PL2303_CONSOLE - Sets up the
        Prolifics PL2303 emulation as a console device
        at /dev/console.

      CONFIG_CDCACM and CONFIG_CDCACM_CONSOLE - Sets up the
        CDC/ACM serial device as a console device at
        dev/console.

      CONFIG_NSH_USBCONSOLE
        If defined, then the an arbitrary USB device may be used
        to as the NSH console.  In this case, CONFIG_NSH_USBCONDEV
        must be defined to indicate which USB device to use as
        the console.

      CONFIG_NSH_USBCONDEV
        If CONFIG_NSH_USBCONSOLE is set to 'y', then CONFIG_NSH_USBCONDEV
        must also be set to select the USB device used to support
        the NSH console.   This should be set to the quoted name of a
        readable/write-able USB driver such as:
        CONFIG_NSH_USBCONDEV="/dev/ttyACM0".

      If there are more than one USB devices, then a USB device
      minor number may also need to be provided:

      CONFIG_NSH_UBSDEV_MINOR
        The minor device number of the USB device.  Default: 0

      If USB tracing is enabled (CONFIG_USBDEV_TRACE), then NSH will
      initialize USB tracing as requested by the following. Default:
      Only USB errors are traced.

      CONFIG_NSH_USBDEV_TRACEINIT
        Show initialization events
      CONFIG_NSH_USBDEV_TRACECLASS
        Show class driver events
      CONFIG_NSH_USBDEV_TRACETRANSFERS
        Show data transfer events
      CONFIG_NSH_USBDEV_TRACECONTROLLER
        Show controller events
      CONFIG_NSH_USBDEV_TRACEINTERRUPTS
        Show interrupt-related events.

  * CONFIG_NSH_CONDEV
      If CONFIG_NSH_CONSOLE is set to 'y', then CONFIG_NSH_CONDEV
      may also be set to select the serial device used to support
      the NSH console.   This should be set to the quoted name of a
      readable/write-able character driver such as:
      CONFIG_NSH_CONDEV="/dev/ttyS1". This is useful, for example,
      to separate the NSH command line from the system console when
      the system console is used to provide debug output.  Default:
      stdin and stdout (probably "/dev/console")

      NOTE: When any other device other than /dev/console is used
      for a user interface, (1) linefeeds (\n) will not be expanded to
      carriage return / linefeeds (\r\n).  You will need to set
      your terminal program to account for this.  And (2) input is
      not automatically echoed so you will have to turn local echo on.

  * CONFIG_NSH_TELNET
      If CONFIG_NSH_TELNET is set to 'y', then a TELENET
      server front-end is selected.  When this option is provided,
      you may log into NuttX remotely using telnet in order to
      access NSH.

  * CONFIG_NSH_ARCHINIT
      Set if your board provides architecture specific initialization
      via the board-specific function nsh_archinitialize().  This
      function will be called early in NSH initialization to allow
      board logic to do such things as configure MMC/SD slots.

  If Telnet is selected for the NSH console, then we must configure
  the resources used by the Telnet daemon and by the Telnet clients.

  * CONFIG_NSH_TELNETD_PORT - The telnet daemon will listen on this
      TCP port number for connections.  Default: 23

  * CONFIG_NSH_TELNETD_DAEMONPRIO - Priority of the Telnet daemon.
      Default: SCHED_PRIORITY_DEFAULT

  * CONFIG_NSH_TELNETD_DAEMONSTACKSIZE - Stack size allocated for the
      Telnet daemon. Default: 2048

  * CONFIG_NSH_TELNETD_CLIENTPRIO- Priority of the Telnet client.
      Default: SCHED_PRIORITY_DEFAULT

  * CONFIG_NSH_TELNETD_CLIENTSTACKSIZE - Stack size allocated for the
      Telnet client. Default: 2048

  One or both of CONFIG_NSH_CONSOLE and CONFIG_NSH_TELNET
  must be defined.  If CONFIG_NSH_TELNET is selected, then there some
  other configuration settings that apply:

  * CONFIG_NET=y
      Of course, networking must be enabled
 
  * CONFIG_NSOCKET_DESCRIPTORS
      And, of course, you must allocate some socket descriptors.

  * CONFIG_NET_TCP=y
      TCP/IP support is required for telnet (as well as various other TCP-related
      configuration settings).

  * CONFIG_NSH_IOBUFFER_SIZE
      Determines the size of the I/O buffer to use for sending/
      receiving TELNET commands/reponses

  * CONFIG_NSH_DHCPC
      Obtain the IP address via DHCP.

  * CONFIG_NSH_IPADDR
      If CONFIG_NSH_DHCPC is NOT set, then the static IP
      address must be provided.

  * CONFIG_NSH_DRIPADDR
      Default router IP address

  * CONFIG_NSH_NETMASK
      Network mask

  * CONFIG_NSH_NOMAC
      Set if your ethernet hardware has no built-in MAC address.
      If set, a bogus MAC will be assigned.

  * CONFIG_NSH_MAX_ROUNDTRIP
     This is the maximum round trip for a response to a ICMP ECHO request.
    It is in units of deciseconds.  The default is 20 (2 seconds).

  If you use DHCPC, then some special configuration network options are
  required.  These include:

  * CONFIG_NET=y
      Of course, networking must be enabled
 
  * CONFIG_NSOCKET_DESCRIPTORS
      And, of course, you must allocate some socket descriptors.

  * CONFIG_NET_UDP=y
      UDP support is required for DHCP (as well as various other UDP-related
      configuration settings)

  * CONFIG_NET_BROADCAST=y
      UDP broadcast support is needed.
 
  * CONFIG_NET_BUFSIZE=650 (or larger)
      Per RFC2131 (p. 9), the DHCP client must be prepared to receive DHCP
      messages of up to 576 bytes (excluding Ethernet, IP, or UDP headers and FCS).

  If CONFIG_NSH_ROMFSETC is selected, then the following additional
  configuration setting apply:

  * CONFIG_NSH_ROMFSMOUNTPT
      The default mountpoint for the ROMFS volume is /etc, but that
      can be changed with this setting.  This must be a absolute path
      beginning with '/'.

  * CONFIG_NSH_INITSCRIPT
      This is the relative path to the startup script within the mountpoint.
      The default is init.d/rcS.  This is a relative path and must not
      start with '/'.

  * CONFIG_NSH_ROMFSDEVNO
      This is the minor number of the ROMFS block device.  The default is 
      '0' corresponding to /dev/ram0.

  * CONFIG_NSH_ROMFSSECTSIZE
      This is the sector size to use with the ROMFS volume.  Since the
      default volume is very small, this defaults to 64 but should be
      increased if the ROMFS volume were to be become large.  Any value
      selected must be a power of 2.

  When the default rcS file used when CONFIG_NSH_ROMFSETC is
  selected, it will mount a FAT FS under /tmp.  The following selections
  describe that FAT FS.

  * CONFIG_NSH_FATDEVNO
      This is the minor number of the FAT FS block device.  The default is 
      '1' corresponding to /dev/ram1.

  * CONFIG_NSH_FATSECTSIZE
      This is the sector size use with the FAT FS. Default is 512.

  * CONFIG_NSH_FATNSECTORS
      This is the number of sectors to use with the FAT FS.  Defalt is
      1024.  The amount of memory used by the FAT FS will be
      CONFIG_NSH_FATSECTSIZE * CONFIG_NSH_FATNSECTORS
      bytes.

  * CONFIG_NSH_FATMOUNTPT
      This is the location where the FAT FS will be mounted.  Default
      is /tmp.

Common Problems
^^^^^^^^^^^^^^^

  Problem:
    Using NSH over serial, the "nsh>" prompt repeats over and over again
    with no serial input.
  Usual Cause:
    NSH over serial needs to use the interrupt driven serial driver
    (drivers/serial/serial.c) not the polled serial driver (drivers/serial/lowconsole.c).
    Make sure that the polled console is disabled in the OS configuration
    file, .config.  That file should have CONFIG_DEV_LOWCONSOLE=n for
    NSH over serial.

  Problem:
    The function 'readline' is undefined.
  Usual Cause:
    The following is missing from your appconfig file:

    CONFIGURED_APPS += system/readline

