######################################################################
#
# genromfs
#
######################################################################

GENROMFS_VERSION:=0.5.2
GENROMFS_SOURCE:=genromfs-$(GENROMFS_VERSION).tar.gz
GENROMFS_BUILD:=$(TOOL_BUILD_DIR)/genromfs-$(GENROMFS_VERSION)

$(GENROMFS_BUILD)/.unpacked : $(GENROMFS_TARBALL)
	$(ZCAT) toolchain/genromfs/$(GENROMFS_SOURCE) | tar -C $(TOOL_BUILD_DIR) $(TAR_OPTIONS) -
	toolchain/patch-kernel.sh $(GENROMFS_BUILD) toolchain/genromfs \*.patch
	touch $@

$(GENROMFS_BUILD)/.compiled : $(GENROMFS_BUILD)/.unpacked
	$(MAKE) -C $(GENROMFS_BUILD)
	touch $@

$(STAGING_DIR)/bin/genromfs: $(GENROMFS_BUILD)/.compiled
	install -m 755 $(GENROMFS_BUILD)/genromfs $(STAGING_DIR)/bin/genromfs

genromfs: $(STAGING_DIR)/bin/genromfs

genromfs-source:

genromfs-clean:
	rm -f $(STAGING_DIR)/bin/genromfs
	(if [ -d $(GENROMFS_BUILD) ]; then $(MAKE) -C $(GENROMFS_BUILD) clean; fi)
	rm -f $(GENROMFS_BUILD)/.compiled

genromfs-dirclean:
	rm -rf $(GENROMFS_BUILD)

ifeq ($(strip $(BR2_PACKAGE_GENROMFS)),y)
TARGETS+=genromfs
endif

