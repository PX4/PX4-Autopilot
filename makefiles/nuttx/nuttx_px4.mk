#
#   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
# What we're going to build.
#
PRODUCT_BUNDLE		 = $(WORK_DIR)firmware.px4
PRODUCT_BIN		 = $(WORK_DIR)firmware.bin
PRODUCT_ELF		 = $(WORK_DIR)firmware.elf
PRODUCT_PARAMXML	 = $(WORK_DIR)/parameters.xml
PRODUCT_AIRFRAMEXML	 = $(WORK_DIR)/airframes.xml

.PHONY:			firmware
firmware:		$(PRODUCT_BUNDLE)

#
# Built product rules
#

$(PRODUCT_BUNDLE):	$(PRODUCT_BIN)
	@$(ECHO) %% Generating $@
ifdef GEN_PARAM_XML
	$(Q) $(PYTHON) $(PX4_BASE)/Tools/px_process_params.py --src-path $(PX4_BASE)/src --board CONFIG_ARCH_BOARD_$(CONFIG_BOARD) --xml
	$(Q) $(PYTHON) $(PX4_BASE)/Tools/px_process_airframes.py -a $(PX4_BASE)/ROMFS/px4fmu_common/init.d/ --board CONFIG_ARCH_BOARD_$(CONFIG_BOARD) --xml
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(BOARD).prototype \
		--git_identity $(PX4_BASE) \
		--parameter_xml $(PRODUCT_PARAMXML) \
		--airframe_xml $(PRODUCT_AIRFRAMEXML) \
		--image $< > $@
else
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(BOARD).prototype \
		--git_identity $(PX4_BASE) \
		--image $< > $@
endif

$(PRODUCT_BIN):		$(PRODUCT_ELF)
	$(call SYM_TO_BIN,$<,$@)

$(PRODUCT_ELF):		$(OBJS) $(MODULE_OBJS) $(LIBRARY_LIBS) $(GLOBAL_DEPS) $(LINK_DEPS) $(MODULE_MKFILES)
	$(call LINK,$@,$(OBJS) $(MODULE_OBJS) $(LIBRARY_LIBS))

#
# Utility rules
#

.PHONY: upload
upload:	$(PRODUCT_BUNDLE) $(PRODUCT_BIN)
	$(Q) $(MAKE) -f $(PX4_MK_DIR)/nuttx/upload.mk \
		METHOD=serial \
		CONFIG=$(CONFIG) \
		BOARD=$(BOARD) \
		BUNDLE=$(PRODUCT_BUNDLE) \
		BIN=$(PRODUCT_BIN)

.PHONY: clean
clean:			$(MODULE_CLEANS)
	@$(ECHO) %% cleaning
	$(Q) $(REMOVE) $(PRODUCT_BUNDLE) $(PRODUCT_BIN) $(PRODUCT_ELF)
	$(Q) $(REMOVE) $(OBJS) $(DEP_INCLUDES) $(EXTRA_CLEANS)
	$(Q) $(RMDIR) $(NUTTX_EXPORT_DIR)

