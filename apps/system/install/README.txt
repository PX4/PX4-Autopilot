
Install Program
===============

    Source: NuttX
    Author: Uros Platise
    Date: 7. May 2011

This application installs XIP application by placing it directly into
the program memory (flash) area into free area and creates a start-up
script into selected directory (i.e. /usr/bin/progname).

Usage:
    install [--stack RequiredStackSpace] [--priority Priority] 
            source-file destination-directory
    
If stackspace is not given default stack space of 4096 Bytes is used.
If priority is not given system default is used.

Additional options:

    --remove destination-file   i.e. install --remove /usr/bin/myapp
    --force                     to replace existing installation
    --start <page>              install app at or after <page>
    --margin <pages>            leave some free space after the kernel
                                Default is 16 pages so kernel may grow.
