######################################################################
#
# sstrip
#
######################################################################

SSTRIP_SOURCE_FILE:=$(TOPDIR)/toolchain/sstrip/sstrip.c

######################################################################
#
# sstrip host
#
######################################################################

SSTRIP_HOST:=$(STAGING_DIR)/bin/$(REAL_GNU_TARGET_NAME)-sstrip

$(SSTRIP_HOST): $(SSTRIP_SOURCE_FILE)
	ln -snf ../../bin/$(REAL_GNU_TARGET_NAME)-sstrip \
		$(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/bin/sstrip
	ln -snf $(REAL_GNU_TARGET_NAME)-sstrip \
		$(STAGING_DIR)/bin/$(GNU_TARGET_NAME)-sstrip
	$(HOSTCC) $(SSTRIP_SOURCE_FILE) -o $(SSTRIP_HOST)

sstrip_host: $(SSTRIP_HOST)

sstrip_host-source: $(SSTRIP_SOURCE_FILE)

sstrip_host-clean:
	rm -f $(SSTRIP_HOST)
	rm -f $(STAGING_DIR)/$(REAL_GNU_TARGET_NAME)/bin/sstrip
	rm -f $(STAGING_DIR)/bin/$(GNU_TARGET_NAME)-sstrip

sstrip_host-dirclean:
	true

######################################################################
#
# sstrip target
#
######################################################################

SSTRIP_TARGET:=$(TARGET_DIR)/usr/bin/sstrip

$(SSTRIP_TARGET): $(SSTRIP_SOURCE_FILE)
	$(TARGET_CC) $(TARGET_CFLAGS) $(SSTRIP_SOURCE_FILE) -o $(SSTRIP_TARGET)

sstrip_target: $(SSTRIP_TARGET)

sstrip_target-source: $(SSTRIP_SOURCE_FILE)

sstrip_target-clean:
	rm -f $(SSTRIP_TARGET)

sstrip_target-dirclean:
	true

#############################################################
#
# Toplevel Makefile options
#
#############################################################

ifeq ($(strip $(BR2_PACKAGE_SSTRIP_HOST)),y)
TARGETS+=sstrip_host
endif

ifeq ($(strip $(BR2_PACKAGE_SSTRIP_TARGET)),y)
TARGETS+=sstrip_target
endif
