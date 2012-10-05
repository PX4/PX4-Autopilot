######################################################################
#
# gdb
#
######################################################################
GDB_VERSION:=$(strip $(subst ",, $(BR2_GDB_VERSION)))

ifeq ($(GDB_VERSION),snapshot)
# Be aware that this changes daily....
GDB_SITE:=ftp://sources.redhat.com/pub/gdb/snapshots/current
GDB_SOURCE:=gdb.tar.bz2
GDB_CAT:=$(BZCAT)
GDB_DIR:=$(TOOL_BUILD_DIR)/gdb-$(GDB_VERSION)
else
GDB_SITE:=http://ftp.gnu.org/gnu/gdb
GDB_SOURCE:=gdb-$(GDB_VERSION).tar.bz2
GDB_CAT:=$(BZCAT)

GDB_DIR:=$(TOOL_BUILD_DIR)/gdb-$(GDB_VERSION)

# NOTE: This option should not be used with gdb versions 6.4 and above.

ifeq ($(GDB_VERSION),6.3)
DISABLE_GDBMI:=--disable-gdbmi
endif
endif

$(DL_DIR)/$(GDB_SOURCE):
	$(WGET) -P $(DL_DIR) $(GDB_SITE)/$(GDB_SOURCE)

gdb-unpacked: $(GDB_DIR)/.unpacked
$(GDB_DIR)/.unpacked: $(DL_DIR)/$(GDB_SOURCE)
	$(GDB_CAT) $(DL_DIR)/$(GDB_SOURCE) | tar -C $(TOOL_BUILD_DIR) $(TAR_OPTIONS) -
ifeq ($(GDB_VERSION),snapshot)
	GDB_REAL_DIR=$(shell \
		tar jtf $(DL_DIR)/$(GDB_SOURCE) | head -1 | cut -d"/" -f1)
	ln -sf $(TOOL_BUILD_DIR)/$(shell tar jtf $(DL_DIR)/$(GDB_SOURCE) | head -1 | cut -d"/" -f1) $(GDB_DIR)
endif
ifneq ($(wildcard toolchain/gdb/$(GDB_VERSION)),)
	toolchain/patch-kernel.sh $(GDB_DIR) toolchain/gdb/$(GDB_VERSION) \*.patch
endif
	$(CONFIG_UPDATE) $(GDB_DIR)
	touch $(GDB_DIR)/.unpacked

gdb-dirclean:
	rm -rf $(GDB_DIR)

######################################################################
#
# gdb target
#
######################################################################

GDB_TARGET_DIR:=$(BUILD_DIR)/gdb-$(GDB_VERSION)-target

GDB_TARGET_CONFIGURE_VARS:= \
	ac_cv_type_uintptr_t=yes \
	gt_cv_func_gettext_libintl=yes \
	ac_cv_func_dcgettext=yes \
	gdb_cv_func_sigsetjmp=yes \
	bash_cv_func_strcoll_broken=no \
	bash_cv_must_reinstall_sighandlers=no \
	bash_cv_func_sigsetjmp=present \
	bash_cv_have_mbstate_t=yes

$(GDB_TARGET_DIR)/.configured: $(GDB_DIR)/.unpacked
	mkdir -p $(GDB_TARGET_DIR)
	(cd $(GDB_TARGET_DIR); \
		gdb_cv_func_sigsetjmp=yes \
		$(TARGET_CONFIGURE_OPTS) \
		CFLAGS_FOR_TARGET="$(TARGET_CFLAGS) $(TARGET_LDFLAGS)" \
		CFLAGS="$(TARGET_CFLAGS) $(TARGET_LDFLAGS)" \
		$(GDB_TARGET_CONFIGURE_VARS) \
		$(GDB_DIR)/configure \
		--build=$(GNU_HOST_NAME) \
		--host=$(REAL_GNU_TARGET_NAME) \
		--target=$(REAL_GNU_TARGET_NAME) \
		--prefix=/usr \
		$(DISABLE_NLS) \
		--without-uiout $(DISABLE_GDBMI) \
		--disable-tui --disable-gdbtk --without-x \
		--disable-sim --enable-gdbserver \
		--without-included-gettext \
		--disable-werror \
	);
ifeq ($(BR2_ENABLE_LOCALE),y)
	-$(SED) "s,^INTL *=.*,INTL = -lintl,g;" $(GDB_DIR)/gdb/Makefile
endif
	touch  $(GDB_TARGET_DIR)/.configured

$(GDB_TARGET_DIR)/gdb/gdb: $(GDB_TARGET_DIR)/.configured
	$(MAKE) CC=$(TARGET_CC) MT_CFLAGS="$(TARGET_CFLAGS)" \
		-C $(GDB_TARGET_DIR)
	$(STRIP) $(GDB_TARGET_DIR)/gdb/gdb

$(TARGET_DIR)/usr/bin/gdb: $(GDB_TARGET_DIR)/gdb/gdb
	install -c -D $(GDB_TARGET_DIR)/gdb/gdb $(TARGET_DIR)/usr/bin/gdb

gdb_target: $(TARGET_DIR)/usr/bin/gdb

gdb_target-source: $(DL_DIR)/$(GDB_SOURCE)

gdb_target-clean:
	@if [ -d $(GDB_DIR)/Makefile ] ; then \
		$(MAKE) -C $(GDB_DIR) clean ; \
	fi;

gdb_target-dirclean:
	rm -rf $(GDB_DIR)

######################################################################
#
# gdbserver
#
######################################################################

GDB_SERVER_DIR:=$(BUILD_DIR)/gdbserver-$(GDB_VERSION)

$(GDB_SERVER_DIR)/.configured: $(GDB_DIR)/.unpacked
	mkdir -p $(GDB_SERVER_DIR)
	(cd $(GDB_SERVER_DIR); \
		$(TARGET_CONFIGURE_OPTS) \
		gdb_cv_func_sigsetjmp=yes \
		$(GDB_DIR)/gdb/gdbserver/configure \
		--build=$(GNU_HOST_NAME) \
		--host=$(REAL_GNU_TARGET_NAME) \
		--target=$(REAL_GNU_TARGET_NAME) \
		--prefix=/usr \
		--exec-prefix=/usr \
		--bindir=/usr/bin \
		--sbindir=/usr/sbin \
		--libexecdir=/usr/lib \
		--sysconfdir=/etc \
		--datadir=/usr/share \
		--localstatedir=/var \
		--mandir=/usr/man \
		--infodir=/usr/info \
		--includedir=$(STAGING_DIR)/include \
		$(DISABLE_NLS) \
		--without-uiout $(DISABLE_GDBMI) \
		--disable-tui --disable-gdbtk --without-x \
		--without-included-gettext \
	);
	touch  $(GDB_SERVER_DIR)/.configured

$(GDB_SERVER_DIR)/gdbserver: $(GDB_SERVER_DIR)/.configured
	$(MAKE) CC=$(TARGET_CC) MT_CFLAGS="$(TARGET_CFLAGS)" \
		-C $(GDB_SERVER_DIR)
	$(STRIP) $(GDB_SERVER_DIR)/gdbserver
$(TARGET_DIR)/usr/bin/gdbserver: $(GDB_SERVER_DIR)/gdbserver
	install -c -D $(GDB_SERVER_DIR)/gdbserver $(TARGET_DIR)/usr/bin/gdbserver

gdbserver: $(TARGET_DIR)/usr/bin/gdbserver

gdbserver-clean:
	@if [ -d $(GDB_SERVER_DIR)/Makefile ] ; then \
		$(MAKE) -C $(GDB_SERVER_DIR) clean ; \
	fi;

gdbserver-dirclean:
	rm -rf $(GDB_SERVER_DIR)

######################################################################
#
# gdb on host
#
######################################################################

GDB_HOST_DIR:=$(TOOL_BUILD_DIR)/gdbhost-$(GDB_VERSION)

$(GDB_HOST_DIR)/.configured: $(GDB_DIR)/.unpacked
	mkdir -p $(GDB_HOST_DIR)
	(cd $(GDB_HOST_DIR); \
		gdb_cv_func_sigsetjmp=yes \
		$(GDB_DIR)/configure \
		--prefix=$(STAGING_DIR) \
		--build=$(GNU_HOST_NAME) \
		--host=$(GNU_HOST_NAME) \
		--target=$(REAL_GNU_TARGET_NAME) \
		$(DISABLE_NLS) \
		--without-uiout $(DISABLE_GDBMI) \
		--disable-tui --disable-gdbtk --without-x \
		--without-included-gettext \
		--enable-threads \
	);
	touch  $(GDB_HOST_DIR)/.configured

$(GDB_HOST_DIR)/gdb/gdb: $(GDB_HOST_DIR)/.configured
	$(MAKE) -C $(GDB_HOST_DIR)
	strip $(GDB_HOST_DIR)/gdb/gdb

$(TARGET_CROSS)gdb: $(GDB_HOST_DIR)/gdb/gdb
	install -c $(GDB_HOST_DIR)/gdb/gdb $(TARGET_CROSS)gdb
	ln -snf ../../bin/$(REAL_GNU_TARGET_NAME)-gdb \
		$(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/bin/gdb
	ln -snf $(REAL_GNU_TARGET_NAME)-gdb \
		$(STAGING_DIR)/bin/$(GNU_TARGET_NAME)-gdb

gdbhost: $(TARGET_CROSS)gdb

gdbhost-clean:
	@if [ -d $(GDB_HOST_DIR)/Makefile ] ; then \
		$(MAKE) -C $(GDB_HOST_DIR) clean ; \
	fi;

gdbhost-dirclean:
	rm -rf $(GDB_HOST_DIR)



#############################################################
#
# Toplevel Makefile options
#
#############################################################
ifeq ($(strip $(BR2_PACKAGE_GDB)),y)
TARGETS+=gdb_target
endif

ifeq ($(strip $(BR2_PACKAGE_GDB_SERVER)),y)
TARGETS+=gdbserver
endif

ifeq ($(strip $(BR2_PACKAGE_GDB_HOST)),y)
TARGETS+=gdbhost
endif
