apps/interpreter/README.txt
===========================

Ficl is a programming language interpreter designed to be embedded into
other systems as a command, macro, and development prototyping language.
Ficl is an acronym for "Forth Inspired Command Language". See
http://ficl.sourceforge.net/

Build Instructions
------------------

Disclaimer:  This installation steps have only been exercised using Ficl
4.1.0.  With new versions you will likely have to make some adjustments
to this instructtions or to the files within this directory.  Think of this
information as "recommendations" -- not necessarily proven instructions.

1. CD to apps/interpreters/ficl

2. Download Ficl: http://sourceforge.net/projects/ficl/files/

3. Uznip the Ficl compressed file.

   For example, 'unzip ficl-4.1.0.zip' will leave the file
   apps/interpreters/ficl/ficl-4.1.0

4. Configure to build Ficl in the apps/interpreters/ficl directory using
   the configure.sh script.

   For example, './configure.sh ficl-4.1.0' will leave the Makefile
   fragment 'Make.srcs' in the ficl build directory.

5. Create your NuttX configuration.  The appconfig file should include
   (1) the path to your application code, and (2) the path to the Ficl
   build directory.  That latter would appear as the following line in
   your appconfig file:

   CONFIGURED_APPS += interpreters/ficl

 6. Configure and build NuttX.  On successful completion, the Ficl objects
    will be available in apps/libapps.a and that NuttX binary will be
    linked against that file.  Of course, Ficl will do nothing unless
    you have written some application code that uses it!
