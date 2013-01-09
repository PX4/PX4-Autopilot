misc/tools/README.txt
=====================

Contents:

  o genromfs-0.5.2.tar.gz
  o kconfig-frontends
    - --program-prefix=
    - kconfig-frontends-3.3.0-1-libintl.patch
    - kconfig-macos.patch
    - kconfig-macos.patch
    - kconfig-frontends for Windows

genromfs-0.5.2.tar.gz
=====================

  This is a snapshot of the genromfs tarball taken from
  http://sourceforge.net/projects/romfs/.  This snapshot is provided to
  assure that a working version of genromfs is always available for NuttX.

  This tool is also include in the buildroot and can be built automatically
  from the buildroot.

kconfig-frontends
=================

  This is a snapshot of the kconfig-frontends version 3.7.0 tarball taken
  from http://ymorin.is-a-geek.org/projects/kconfig-frontends.

  General build instructions:

    cd kconfig-frontends
    ./configure
    make
    make install

  To suppress the nconf and the graphical interfaces which are not used by
  NuttX:

    ./configure --disable-gconf --disable-qconf
    make
    make install

  To suppress the graphical interfaces, use static libraries, and disable
  creation of other utilities:

    ./configure --disable-shared --enable-static --disable-gconf --disable-qconf --disable-nconf --disable-utils
    make
    make install

  You may require root privileges to 'make install'.

--program-prefix=
-----------------

  Beginning somwhere between version 3.3.0 and 3.6.0, the prefix was added
  to the kconfig-frontends tools.  The default prefix is kconfig-.  So,
  after 3.3.0, conf becomes kconfig-conf, mconf becomes kconfig-mconf, etc.
  All of the NuttX documentation, Makefiles, scripts have been updated to
  used this default prefix.

  This introduces an incompatibility with the 3.3.0 version.  In the 3.6.0
  timeframe, the configure argument --program-prefix= was added to
  eliminated the kconfig- prefix.  This, however, caused problems when we
  got to the 3.7.0 version which generates a binary called kconfig-diff
  (installed at /usr/local/bin).  Without the prefix, may conflict with
  the standard diff utility (at /bin), depending upon how your PATH
  variable is configured.  Because of this, we decided to "bite the bullet"
  and use the standard prefix at 3.7.0 and later.

  This problem could probably also be avoided using --disable-utils.

kconfig-frontends-3.3.0-1-libintl.patch
---------------------------------------

  The above build instructions did not work for me under my Cygwin
  installation with kconfig-frontends-3.3.0.  This patch is a awful hack
  but will successfully build 'kconfig-mconf' under Cygwin.

    cat kconfig-frontends-3.3.0-1-libintl.patch | patch -p0
    cd kconfig-frontends-3.3.0-1
    ./configure --disable-gconf --disable-qconf
    make
    make install

  See: http://ymorin.is-a-geek.org/hg/kconfig-frontends/file/tip/docs/known-issues.txt

  Update: Version 3.6.0 (and above) will build on Cygwin with no patches: 

    http://ymorin.is-a-geek.org/download/kconfig-frontends/

kconfig-macos.patch
-------------------

  This is a patch to make the kconfig-frontends-3.3.0 build on Mac OS X.

  To build the conf and mconf frontends, use the following commands:

    ./configure --disable-shared --enable-static --disable-gconf --disable-qconf --disable-nconf --disable-utils
    make
    make install

kconfig-frontends for Windows
-----------------------------

From http://tech.groups.yahoo.com/group/nuttx/message/2900:

"The build was quite simple:

I used mingw installer and I had to install two packages that the
automated mingw setup does not bring by default:

  * mingw-get update
  * mingw-get install mingw32-pdcurses mingw32-libpdcurses
  * mingw-get install msys-regex msys-libregex

(grep the output of mingw-get list if I got the names wrong)

Then I had to change some things in mconf code, it was quite simple to
understand the make errors.

  * The first of them is to disable any use of uname() in symbol.c and
    replace the uname output by a constant string value (I used MINGW32-MSYS),

  * The second one is related to the second parameter to mkdir() that has
    to disappear for windows (we don't care about folder rights) in confdata.c;

  * And the last one of them involves #undef bool in dialog.h before including
    curses.h (CURSES_LOC), around line 30.

I wrapped all of my changes in #if(n)def __MINGW32__, but that is not
sufficient to make that work everywhere, I think.

So mconf itself has some portability issues that shall be managed in a
cleaner way, what I did was just hacks, I don't think they are
acceptable by mconf upstream maintainers.

Here is the magic incantation to get the whole thing working. It seems
that the configure script is not so good and does not bring the required
bits to link libregex.

  CFLAGS="-I/mingw/include -I/usr/include" LDFLAGS="-Bstatic -L/mingw/lib
         -L/usr/lib -lregex" ./configure --enable-frontends=mconf --enable-static
         --disable-shared

So the message I want to pass is that native "make menuconfig" in
windows IS POSSIBLE, I have done it in a few minutes."

"Oops, forgot something, I had to bring a gperf binary from the gnuwin32 project."

- Sebastien Lorquet
