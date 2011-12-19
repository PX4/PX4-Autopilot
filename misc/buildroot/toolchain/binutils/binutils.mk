#############################################################
#
# build binutils for use on the host system
#
#############################################################
BINUTILS_VERSION:=$(strip $(subst ",, $(BR2_BINUTILS_VERSION)))
#"))

EXTRA_BINUTILS_CONFIG_OPTIONS=$(strip $(subst ",, $(BR2_EXTRA_BINUTILS_CONFIG_OPTIONS)))
#"))
BINUTILS_SITE:=ftp://ftp.gnu.org/gnu/binutils

# NOTE: Unlike the original buildroot binutils.mk, this version always relies on
# the system libgmp and libmpfr which must be installed for certain binutils versions.

BINUTILS_SOURCE:=binutils-$(BINUTILS_VERSION).tar.bz2
BINUTILS_DIR:=$(TOOL_BUILD_DIR)/binutils-$(BINUTILS_VERSION)
BINUTILS_CAT:=$(BZCAT)

BINUTILS_DIR1:=$(TOOL_BUILD_DIR)/binutils-$(BINUTILS_VERSION)-build

$(DL_DIR)/$(BINUTILS_SOURCE):
	mkdir -p $(DL_DIR)
	$(WGET) -P $(DL_DIR) $(BINUTILS_SITE)/$(BINUTILS_SOURCE)

binutils-unpacked: $(BINUTILS_DIR)/.unpacked
$(BINUTILS_DIR)/.unpacked: $(DL_DIR)/$(BINUTILS_SOURCE)
	mkdir -p $(TOOL_BUILD_DIR)
	$(BINUTILS_CAT) $(DL_DIR)/$(BINUTILS_SOURCE) | tar -C $(TOOL_BUILD_DIR) $(TAR_OPTIONS) -
	$(CONFIG_UPDATE) $(BINUTILS_DIR)
	touch $@

$(BINUTILS_DIR)/.patched: $(BINUTILS_DIR)/.unpacked
	# Apply appropriate binutils patches.
	toolchain/patch-kernel.sh $(BINUTILS_DIR) toolchain/binutils/$(BINUTILS_VERSION) \*.patch
	touch $@

$(BINUTILS_DIR1)/.configured: $(BINUTILS_DIR)/.patched
	mkdir -p $(BINUTILS_DIR1)
	(cd $(BINUTILS_DIR1); \
		CC="$(HOSTCC)" \
		$(BINUTILS_DIR)/configure \
		--prefix=$(STAGING_DIR) \
		--build=$(GNU_HOST_NAME) \
		--host=$(GNU_HOST_NAME) \
		--target=$(REAL_GNU_TARGET_NAME) \
		$(DISABLE_NLS) \
		$(MULTILIB) \
		--disable-werror \
		$(SOFT_FLOAT_CONFIG_OPTION) \
		$(EXTRA_BINUTILS_CONFIG_OPTIONS));
	touch $@

$(BINUTILS_DIR1)/binutils/objdump: $(BINUTILS_DIR1)/.configured
	$(MAKE) -C $(BINUTILS_DIR1) all

# Make install will put gettext data in staging_dir/share/locale.
# Unfortunatey, it isn't configureable.
$(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/bin/ld: $(BINUTILS_DIR1)/binutils/objdump
	$(MAKE) -C $(BINUTILS_DIR1) install

binutils: dependencies $(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/bin/ld

binutils-source: $(DL_DIR)/$(BINUTILS_SOURCE)

binutils-clean:
	rm -f $(STAGING_DIR)/bin/$(REAL_GNU_TARGET_NAME)*
	-$(MAKE) -C $(BINUTILS_DIR1) clean

binutils-dirclean:
	rm -rf $(BINUTILS_DIR1)



#############################################################
#
# build binutils for use on the target system
#
#############################################################
BINUTILS_DIR2:=$(BUILD_DIR)/binutils-$(BINUTILS_VERSION)-target
$(BINUTILS_DIR2)/.configured: $(BINUTILS_DIR)/.patched
	mkdir -p $(BINUTILS_DIR2)
	(cd $(BINUTILS_DIR2); \
		CC_FOR_BUILD="$(HOSTCC)" \
		PATH=$(TARGET_PATH) \
		CFLAGS="$(TARGET_CFLAGS)" \
		CFLAGS_FOR_BUILD="-O2 -g" \
		$(BINUTILS_DIR)/configure \
		--prefix=/usr \
		--exec-prefix=/usr \
		--build=$(GNU_HOST_NAME) \
		--host=$(REAL_GNU_TARGET_NAME) \
		--target=$(REAL_GNU_TARGET_NAME) \
		$(DISABLE_NLS) \
		$(MULTILIB) \
		$(BINUTILS_TARGET_CONFIG_OPTIONS) \
		--disable-werror \
		$(SOFT_FLOAT_CONFIG_OPTION) );
	touch $@

$(BINUTILS_DIR2)/binutils/objdump: $(BINUTILS_DIR2)/.configured
	PATH=$(TARGET_PATH) \
	$(MAKE) -C $(BINUTILS_DIR2) all

$(TARGET_DIR)/usr/bin/ld: $(BINUTILS_DIR2)/binutils/objdump
	PATH=$(TARGET_PATH) \
	$(MAKE) DESTDIR=$(TARGET_DIR) \
		tooldir=/usr build_tooldir=/usr \
		-C $(BINUTILS_DIR2) install
	#rm -rf $(TARGET_DIR)/share/locale $(TARGET_DIR)/usr/info \
	#	$(TARGET_DIR)/usr/man $(TARGET_DIR)/usr/share/doc
	-$(STRIP) $(TARGET_DIR)/usr/$(REAL_GNU_TARGET_NAME)/bin/* > /dev/null 2>&1
	-$(STRIP) $(TARGET_DIR)/usr/bin/* > /dev/null 2>&1

binutils_target: $(TARGET_DIR)/usr/bin/ld

binutils_target-clean:
	(cd $(TARGET_DIR)/usr/bin; \
		rm -f addr2line ar as gprof ld nm objcopy \
		      objdump ranlib readelf size strings strip)
	rm -f $(TARGET_DIR)/bin/$(REAL_GNU_TARGET_NAME)*
	-$(MAKE) -C $(BINUTILS_DIR2) clean

binutils_target-dirclean:
	rm -rf $(BINUTILS_DIR2)
