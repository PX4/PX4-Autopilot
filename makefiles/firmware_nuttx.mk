#
#   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

#
# Built products
#

DESIRED_FIRMWARES 	 = $(foreach config,$(CONFIGS),$(IMAGE_DIR)$(config).px4)
STAGED_FIRMWARES	 = $(foreach config,$(KNOWN_CONFIGS),$(IMAGE_DIR)$(config).px4)
FIRMWARES		 = $(foreach config,$(KNOWN_CONFIGS),$(BUILD_DIR)$(config).build/firmware.px4)

all:	$(DESIRED_FIRMWARES)

#
# Copy FIRMWARES into the image directory.
#
# XXX copying the .bin files is a hack to work around the PX4IO uploader
#     not supporting .px4 files, and it should be deprecated onced that
#     is taken care of.
#
$(STAGED_FIRMWARES): $(IMAGE_DIR)%.px4: $(BUILD_DIR)%.build/firmware.px4
	@$(ECHO) %% Copying $@
	$(Q) $(COPY) $< $@
	$(Q) $(COPY) $(patsubst %.px4,%.bin,$<) $(patsubst %.px4,%.bin,$@)

#
# Generate FIRMWARES.
#
.PHONY: $(FIRMWARES)
$(BUILD_DIR)%.build/firmware.px4: config   = $(patsubst $(BUILD_DIR)%.build/firmware.px4,%,$@)
$(BUILD_DIR)%.build/firmware.px4: work_dir = $(BUILD_DIR)$(config).build/
$(FIRMWARES): $(BUILD_DIR)%.build/firmware.px4:	generateuorbtopicheaders checksubmodules
	@$(ECHO) %%%%
	@$(ECHO) %%%% Building $(config) in $(work_dir)
	@$(ECHO) %%%%
	$(Q) $(MKDIR) -p $(work_dir)
	$(Q)+ $(MAKE) -r -C $(work_dir) \
		-f $(PX4_MK_DIR)firmware.mk \
		CONFIG=$(config) \
		WORK_DIR=$(work_dir) \
		$(FIRMWARE_GOAL)

#
# Make FMU firmwares depend on the corresponding IO firmware.
#
# This is a pretty vile hack, since it hard-codes knowledge of the FMU->IO dependency
# and forces the _default config in all cases. There has to be a better way to do this...
#
FMU_VERSION		 = $(patsubst px4fmu-%,%,$(word 1, $(subst _, ,$(1))))
define FMU_DEP
$(BUILD_DIR)$(1).build/firmware.px4: $(IMAGE_DIR)px4io-$(call FMU_VERSION,$(1))_default.px4
endef
FMU_CONFIGS		:= $(filter px4fmu%,$(CONFIGS))
$(foreach config,$(FMU_CONFIGS),$(eval $(call FMU_DEP,$(config))))

#
# Build the NuttX export archives.
#
# Note that there are no explicit dependencies extended from these
# archives. If NuttX is updated, the user is expected to rebuild the
# archives/build area manually. Likewise, when the 'archives' target is
# invoked, all archives are always rebuilt.
#
# XXX Should support fetching/unpacking from a separate directory to permit
#     downloads of the prebuilt archives as well...
#
NUTTX_ARCHIVES		 = $(foreach board,$(BOARDS),$(ARCHIVE_DIR)$(board).export)
.PHONY:			archives
archives:		checksubmodules $(NUTTX_ARCHIVES)

$(ARCHIVE_DIR)%.export:	board = $(notdir $(basename $@))
$(ARCHIVE_DIR)%.export:	configuration = nsh
$(NUTTX_ARCHIVES): $(ARCHIVE_DIR)%.export: $(NUTTX_SRC)
	@$(ECHO) %% Configuring NuttX for $(board)
	$(Q) (cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(Q)+ $(MAKE) -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)/configs && $(COPYDIR) $(PX4_BASE)nuttx-configs/$(board) .)
	$(Q) (cd $(NUTTX_SRC)tools && ./configure.sh $(board)/$(configuration))
	@$(ECHO) %% Exporting NuttX for $(board)
	$(Q)+ $(MAKE) -C $(NUTTX_SRC) -r $(MQUIET) CONFIG_ARCH_BOARD=$(board) export
	$(Q) $(MKDIR) -p $(dir $@)
	$(Q) $(COPY) $(NUTTX_SRC)nuttx-export.zip $@
	$(Q) (cd $(NUTTX_SRC)/configs && $(RMDIR) $(board))

#
# The user can run the NuttX 'menuconfig' tool for a single board configuration with
# make BOARDS=<boardname> menuconfig
#
ifeq ($(MAKECMDGOALS),menuconfig)
ifneq ($(words $(BOARDS)),1)
$(error BOARDS must specify exactly one board for the menuconfig goal)
endif
BOARD			 = $(BOARDS)
menuconfig: $(NUTTX_SRC)
	@$(ECHO) %% Configuring NuttX for $(BOARD)
	$(Q) (cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(Q) $(MAKE) -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)/configs && $(COPYDIR) $(PX4_BASE)nuttx-configs/$(BOARD) .)
	$(Q) (cd $(NUTTX_SRC)tools && ./configure.sh $(BOARD)/nsh)
	@$(ECHO) %% Running menuconfig for $(BOARD)
	$(Q) $(MAKE) -C $(NUTTX_SRC) -r $(MQUIET) menuconfig
	@$(ECHO) %% Saving configuration file
	$(Q)$(COPY) $(NUTTX_SRC).config $(PX4_BASE)nuttx-configs/$(BOARD)/nsh/defconfig
else
menuconfig:
	@$(ECHO) ""
	@$(ECHO) "The menuconfig goal must be invoked without any other goal being specified"
	@$(ECHO) ""
endif

$(NUTTX_SRC): checksubmodules

$(UAVCAN_DIR):
	$(Q) (./Tools/check_submodules.sh)

