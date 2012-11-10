misc/tools/README.txt
=====================

genromfs-0.5.2.tar.gz

  This is a snapshot of the genromfs tarball taken from
  http://sourceforge.net/projects/romfs/.  This snapshot is provided to
  assure that a working version of genromfs is always available for NuttX.

  This tool is also include in the buildroot and can be built automatically
  from the buildroot.

kconfig-frontends

  This is a snapshot of the kconfig-frontends version 3.6.0 tarball taken
  from http://ymorin.is-a-geek.org/projects/kconfig-frontends.

  General build instructions:

    cd kconfig-frontends
    ./configure
    make
    make install

  To suppress the graphical interfaces:

    ./configure --disable-gconf --disable-qconf
    make
    make install

kconfig-frontends-3.3.0-1-libintl.patch

  The above build instructions did not work for me under my Cygwin
  installation with kconfig-frontends-4.4.0.  This patch is a awful hack
  but will successfully build 'mconf' under Cygwin.

    cat kconfig-frontends-3.3.0-1-libintl.patch | patch -p0
    cd kconfig-frontends-3.3.0-1
    ./configure --disable-gconf --disable-qconf
    make
    make install

  See: http://ymorin.is-a-geek.org/hg/kconfig-frontends/file/tip/docs/known-issues.txt

  Update: According to the release notes, version 3.6.0 (and above)
  will build on Cygwin with no patches: 

    http://ymorin.is-a-geek.org/download/kconfig-frontends/

kconfig-macos.path

  This is a patch to make the kconfig-frontends-3.3.0 build on Mac OS X.
