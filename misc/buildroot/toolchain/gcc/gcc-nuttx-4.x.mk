# Makefile for to build a gcc/nuttx toolchain
#
# Copyright (C) 2002-2003 Erik Andersen <andersen@uclibc.org>
# Copyright (C) 2004 Manuel Novoa III <mjn3@uclibc.org>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

# sysroot support works with gcc >= 4.2.0 only
ifeq ($(BR2_GCC_SUPPORTS_SYSROOT),y)

GCC_OFFICIAL_VER:=$(GCC_VERSION)
GCC_SITE:=http://ftp.gnu.org/gnu/gcc/gcc-$(GCC_VERSION)
#GCC_SITE:=ftp://ftp.ibiblio.org/pub/mirrors/gnu/ftp/gnu/gcc/gcc-$(GCC_OFFICIAL_VER)

GCC_SOURCE:=gcc-$(GCC_OFFICIAL_VER).tar.bz2
GCC_DIR:=$(TOOL_BUILD_DIR)/gcc-$(GCC_OFFICIAL_VER)
GCC_CAT:=$(BZCAT)
GCC_STRIP_HOST_BINARIES:=true

# gcc 4.6.x quadmath requires wchar
ifneq ($(BR2_TOOLCHAIN_BUILDROOT_WCHAR),y)
GCC_QUADMATH=--disable-libquadmath
endif

#############################################################
#
# Setup some initial stuff
#
#############################################################

GCC_TARGET_LANGUAGES:=c

ifeq ($(BR2_INSTALL_LIBSTDCPP),y)
GCC_TARGET_LANGUAGES:=$(GCC_TARGET_LANGUAGES),c++
endif

ifeq ($(BR2_INSTALL_LIBGCJ),y)
GCC_TARGET_LANGUAGES:=$(GCC_TARGET_LANGUAGES),java
endif

ifeq ($(BR2_INSTALL_OBJC),y)
GCC_TARGET_LANGUAGES:=$(GCC_TARGET_LANGUAGES),objc
endif

ifeq ($(BR2_INSTALL_FORTRAN),y)
GCC_TARGET_LANGUAGES:=$(GCC_TARGET_LANGUAGES),fortran
endif

GCC_SHARED_LIBGCC:=--disable-shared

ifneq ($(BR2_ENABLE_LOCALE),y)
GCC_ENABLE_CLOCALE:=--disable-clocale
endif

#############################################################
#
# build the gcc compiler
#
#############################################################
GCC_BUILD_DIR:=$(TOOL_BUILD_DIR)/gcc-$(GCC_VERSION)-build

$(DL_DIR)/$(GCC_SOURCE):
	mkdir -p $(DL_DIR)
	$(WGET) -P $(DL_DIR) $(GCC_SITE)/$(GCC_SOURCE)

gcc-unpacked: $(GCC_DIR)/.unpacked
$(GCC_DIR)/.unpacked: $(DL_DIR)/$(GCC_SOURCE)
	mkdir -p $(TOOL_BUILD_DIR)
	$(GCC_CAT) $(DL_DIR)/$(GCC_SOURCE) | tar -C $(TOOL_BUILD_DIR) $(TAR_OPTIONS) -
	$(CONFIG_UPDATE) $(GCC_DIR)
	touch $@

gcc-patched: $(GCC_DIR)/.patched
$(GCC_DIR)/.patched: $(GCC_DIR)/.unpacked
	# Apply any files named gcc-*.patch from the source directory to gcc
	toolchain/patch-kernel.sh $(GCC_DIR) toolchain/gcc/$(GCC_VERSION) \*.patch

	# Note: The soft float situation has improved considerably with gcc 3.4.x.
	# We can dispense with the custom spec files, as well as libfloat for the arm case.
	# However, we still need a patch for arm.  There's a similar patch for gcc 3.3.x
	# which needs to be integrated so we can kill of libfloat for good, except for
	# anyone (?) who might still be using gcc 2.95.  mjn3
ifeq ($(BR2_SOFT_FLOAT),y)
ifeq ("$(strip $(ARCH))","arm")
	toolchain/patch-kernel.sh $(GCC_DIR) toolchain/gcc/$(GCC_VERSION) arm-softfloat.patch.conditional
endif
ifeq ("$(strip $(ARCH))","armeb")
	toolchain/patch-kernel.sh $(GCC_DIR) toolchain/gcc/$(GCC_VERSION) arm-softfloat.patch.conditional
endif
endif
	touch $@

$(GCC_BUILD_DIR)/.configured: $(GCC_DIR)/.patched
	mkdir -p $(GCC_BUILD_DIR)
	# Important!  Required for limits.h to be fixed.
	ln -snf ../include/ $(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/sys-include
	(cd $(GCC_BUILD_DIR); rm -rf config.cache; PATH=$(TARGET_PATH) \
		CC="$(HOSTCC)" \
		$(GCC_DIR)/configure \
		--prefix=$(STAGING_DIR) \
		--build=$(GNU_HOST_NAME) \
		--host=$(GNU_HOST_NAME) \
		--target=$(REAL_GNU_TARGET_NAME) \
		--enable-languages=$(GCC_TARGET_LANGUAGES) \
		--disable-__cxa_atexit \
		--disable-libssp \
		--enable-target-optspace \
		--with-gnu-ld \
		$(GCC_SHARED_LIBGCC) \
		$(DISABLE_NLS) \
		$(THREADS) \
		$(MULTILIB) \
		$(SOFT_FLOAT_CONFIG_OPTION) \
		$(GCC_WITH_ABI) $(GCC_WITH_ARCH) $(GCC_WITH_TUNE) $(GCC_WITH_MODE) \
		$(GCC_USE_SJLJ_EXCEPTIONS) \
		$(DISABLE_LARGEFILE) \
		$(EXTRA_GCC_CONFIG_OPTIONS));
	touch $@

$(GCC_BUILD_DIR)/.compiled: $(GCC_BUILD_DIR)/.configured
	PATH=$(TARGET_PATH) $(MAKE) -C $(GCC_BUILD_DIR) all
	touch $@

$(GCC_BUILD_DIR)/.installed: $(GCC_BUILD_DIR)/.compiled
	PATH=$(TARGET_PATH) $(MAKE) -C $(GCC_BUILD_DIR) install
	if [ -d "$(STAGING_DIR)/lib64" ] ; then \
		if [ ! -e "$(STAGING_DIR)/lib" ] ; then \
			mkdir "$(STAGING_DIR)/lib" ; \
		fi ; \
		mv "$(STAGING_DIR)/lib64/"* "$(STAGING_DIR)/lib/" ; \
		rmdir "$(STAGING_DIR)/lib64" ; \
	fi
	# Strip the host binaries
ifeq ($(GCC_STRIP_HOST_BINARIES),true)
	-strip --strip-all -R .note -R .comment $(STAGING_DIR)/bin/*
endif
	# Make sure we have 'cc'.
	if [ ! -e $(STAGING_DIR)/bin/$(REAL_GNU_TARGET_NAME)-cc ] ; then \
		ln -snf $(REAL_GNU_TARGET_NAME)-gcc \
			$(STAGING_DIR)/bin/$(REAL_GNU_TARGET_NAME)-cc ; \
	fi;
	if [ ! -e $(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/bin/cc ] ; then \
		ln -snf gcc $(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/bin/cc ; \
	fi;
	# Set up the symlinks to enable lying about target name.
	set -e; \
	(cd $(STAGING_DIR); \
		if [ "$(REAL_GNU_TARGET_NAME)" != "$(GNU_TARGET_NAME)" ]; then \
			ln -snf $(REAL_GNU_TARGET_NAME) $(GNU_TARGET_NAME); \
			cd bin; \
			for app in $(REAL_GNU_TARGET_NAME)-* ; do \
				ln -snf $${app} \
				$(GNU_TARGET_NAME)$${app##$(REAL_GNU_TARGET_NAME)}; \
			done; \
		fi; \
	);
	#
	# Ok... that's enough of that.
	#
	touch $@

$(GCC_BUILD_DIR)/.libs_installed: $(GCC_BUILD_DIR)/.installed
ifeq ($(BR2_INSTALL_LIBSTDCPP),y)
	# We have disabled building of libstdc++ for NuttX
	#-cp -dpf $(STAGING_DIR)/lib/libstdc++.so* $(TARGET_DIR)/lib/
endif
ifeq ($(BR2_INSTALL_LIBGCJ),y)
	-cp -dpf $(STAGING_DIR)/lib/libgcj.so* $(TARGET_DIR)/lib/
	-cp -dpf $(STAGING_DIR)/lib/lib-org-w3c-dom.so* $(TARGET_DIR)/lib/
	-cp -dpf $(STAGING_DIR)/lib/lib-org-xml-sax.so* $(TARGET_DIR)/lib/
	-mkdir -p $(TARGET_DIR)/usr/lib/security
	-cp -dpf $(STAGING_DIR)/usr/lib/security/libgcj.security $(TARGET_DIR)/usr/lib/security/
	-cp -dpf $(STAGING_DIR)/usr/lib/security/classpath.security $(TARGET_DIR)/usr/lib/security/
endif
	touch $@

gcc: binutils $(LIBFLOAT_TARGET) \
	$(GCC_BUILD_DIR)/.installed $(GCC_BUILD_DIR)/.libs_installed \
	$(GCC_TARGETS)

gcc-source: $(DL_DIR)/$(GCC_SOURCE)

gcc-clean:
	rm -rf $(GCC_BUILD_DIR)
	for prog in cpp gcc gcc-[0-9]* protoize unprotoize gcov gccbug cc; do \
	    rm -f $(STAGING_DIR)/bin/$(REAL_GNU_TARGET_NAME)-$$prog \
	    rm -f $(STAGING_DIR)/bin/$(GNU_TARGET_NAME)-$$prog; \
	done

gcc-dirclean:
	rm -rf $(GCC_BUILD_DIR)

ifeq ($(strip $(BR2_PACKAGE_GCC)),y)
TARGETS += gcc
endif
endif
