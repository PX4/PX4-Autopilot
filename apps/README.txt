Application Folder
==================

Contents
--------

  General
  Directory Location
  Named Applications
  Named Startup main() function
  NuttShell (NSH) Built-In Commands
  Synchronous Built-In Commands
  Application Configuration File
  Example Named Application
  Building NuttX with Board-Specific Pieces Outside the Source Tree

General
-------
This folder provides various applications found in sub-directories.  These
applications are not inherently a part of NuttX but are provided you help
you develop your own applications.  The apps/ directory is a "break away"
part of the configuration that you may chose to use or not.

Directory Location
------------------
The default application directory used by the NuttX build should be named
apps/ (or apps-x.y/ where x.y is the NuttX version number).  This apps/
directoy should appear in the directory tree at the same level as the
NuttX directory.  Like:

 .
 |- nuttx
 |
 `- apps

If all of the above conditions are TRUE, then NuttX will be able to
find the application directory.  If your application directory has a 
different name or is location at a different position, then you will
have to inform the NuttX build system of that location.  There are several
ways to do that:

1) You can define CONFIG_APPS_DIR to be the full path to your application
   directory in the NuttX configuration file.
2) You can provide the path to the application directory on the command line
   like:  make APPDIR=<path> or make CONFIG_APPS_DIR=<path>
3) When you configure NuttX using tools/configure.sh, you can provide that
   path to the application directory on the configuration command line
   like: ./configure.sh -a <app-dir> <board-name>/<config-name>

Named Applications
------------------
NuttX also supports applications that can be started using a name string.
In this case, zpplication entry points with their requirements are gathered
together in two files:

  - namedapp/namedapp_proto.h  Entry points, prototype function
  - namedapp/namedapp_list.h  Application specific information and requirements

The build occurs in several phases as different build targets are executed:
(1) context, (2) depend, and (3) default (all). Application information is
collected during the make context build phase.

To execute an application function:

  exec_namedapp() is defined in the nuttx/include/apps/apps.h 

NuttShell (NSH) Built-In Commands
---------------------------------
One use of named applications is to provide a way of invoking your custom
application through the NuttShell (NSH) command line.  NSH will support
a seamless method invoking the applications, when the following option is
enabled in the NuttX configuration file:

  CONFIG_NSH_BUILTIN_APPS=y

Applications registered in the apps/namedapp/namedapp_list.h file will then
be accessible from the NSH command line.  If you type 'help' at the NSH
prompt, you will see a list of the registered commands.

Synchronous Built-In Commands
-----------------------------
By default, built-in commands started from the NSH command line will run
asynchronously with NSH.  If you want to force NSH to execute commands
then wait for the command to execute, you can enable that feature by
adding the following to the NuttX configuration file:

CONFIG_SCHED_WAITPID=y

The configuration option enables support for the waitpid() RTOS interface.
When that interface is enabled, NSH will use it to wait, sleeping until
the built-in command executes to completion.

Of course, even with CONFIG_SCHED_WAITPID=y defined, specific commands
can still be forced to run asynchronously by adding the ampersand (&)
after the NSH command.

Application Configuration File
------------------------------
A special configuration file is used to configure which applications
are to be included in the build.  The source for this file is 
configs/<board>/<configuration>/appconfig.  The existence of the appconfig
file in the board configuration directory is sufficient to enable building
of applications.

The appconfig file is copied into the apps/ directory as .config when
NuttX is configured.  .config is included in the toplevel apps/Makefile.
As a minimum, this configuration file must define files to add to the
CONFIGURED_APPS list like:

  CONFIGURED_APPS  += vsn/hello vsn/poweroff vsn/jvm

Named Start-Up main() function
------------------------------
A named application can even be used as the main, start-up entry point
into your embedded software.  When the user defines this option in
the NuttX configuration file:

  CONFIG_BUILTIN_APP_START=<application name>
  
that application shall be invoked immediately after system starts
*instead* of the normal, default "user_start" entry point.
Note that <application name> must be provided as: "hello", 
will call:

  int hello_main(int argc, char *argv[])

Example Named Application
-------------------------
An example application skeleton can be found under the vsn/hello
sub-directory.  This example shows how a named application can be added
to the project. One must define:

 1. create sub-directory as: appname
 2. provide entry point: appname_main()
 3. set the requirements in the file: Makefile, specially the lines:

  APPNAME    = appname
  PRIORITY  = SCHED_PRIORITY_DEFAULT
  STACKSIZE  = 768
  ASRCS    = asm source file list as a.asm b.asm ...
  CSRCS    = C source file list as foo1.c foo2.c ..

 4. add application in the apps/.config

Building NuttX with Board-Specific Pieces Outside the Source Tree
-----------------------------------------------------------------

Q: Has anyone come up with a tidy way to build NuttX with board-
   specific pieces outside the source tree?
A: Here are four:

   1) There is a make target called 'make export'. It will build
      NuttX, then bundle all of the header files, libaries, startup
      objects, and other build components into a .zip file. You
      can can move that .zip file into any build environment you
      want. You even build NuttX under a DOS CMD window.

      This make target is documented in the top level nuttx/README.txt.

   2) You can replace the entire apps/ directory. If there is
      nothing in the apps/ directory that you need, you can define
      CONFIG_APPS_DIR in your .config file so that it points to a
      different, custom application directory.

      You can copy any pieces that you like from the old apps/directory
      to your custom apps directory as necessary.

      This is documented in NuttX/configs/README.txt and
      nuttx/Documentation/NuttxPortingGuide.html (Online at
      http://nuttx.sourceforge.net/NuttxPortingGuide.html#apndxconfigs
      under Build options). And in the apps/README.txt file.

   3) If you like the random collection of stuff in the apps/ directory
      but just want to expand the existing components with your own,
      external sub-directory then there is an easy way to that too:
      You just create the sympolic link at apps/external that
      redirects to your application sub-directory. The apps/Makefile
      will always automatically check for the existence of an
      apps/external directory and if it exists, it will automatically
      incorporate it into the build.

      This feature of the apps/Makefile is documented only here.

      You can, for example, create a script called install.sh that
      installs a custom application, configuration, and board specific
      directory:

      a) Copy 'MyBoard' directory to configs/MyBoard.
      b) At a symbolic link to MyApplication at apps/external
      c) Configure NuttX (usually by:
      
         tools/configure.sh MyBoard/MyConfiguration

         or simply by copying defconfig->nutt/.config,
         setenv.sh->nuttx/setenv.sh, Make.defs->nuttx/Make.defs,
         appconfig->apps/.config

   4) Add any link to apps/

      a) Add symbolic links apps/ to as many other directories as you
         want.
      b) Then just add the (relative) paths to the links in your
         appconfig file (that becomes the apps/.config file).

      That is basically the same as my option #3 but doesn't use the
      magic 'external' link. The toplevel apps/Makefile will always
      to build whatever in finds in the apps/.config file (plus the
      external link if present).
