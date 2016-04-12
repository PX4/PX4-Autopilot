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
# Makefile for PX4 Linux based firmware images.
#

################################################################################
# Build rules
################################################################################

#
# What we're going to build.
#
PRODUCT_SHARED_LIB	= $(WORK_DIR)firmware.a
PRODUCT_SHARED_PRELINK	= $(WORK_DIR)firmware.o

.PHONY:			firmware
firmware:		$(PRODUCT_SHARED_LIB) $(WORK_DIR)mainapp

#
# Built product rules
#

$(PRODUCT_SHARED_PRELINK):	$(OBJS) $(MODULE_OBJS) $(LIBRARY_LIBS) $(GLOBAL_DEPS) $(LINK_DEPS) $(MODULE_MKFILES)
	$(call PRELINKF,$@,$(OBJS) $(MODULE_OBJS) $(LIBRARY_LIBS))

$(PRODUCT_SHARED_LIB):		$(PRODUCT_SHARED_PRELINK)
	$(call LINK_A,$@,$(PRODUCT_SHARED_PRELINK))

$(WORK_DIR)apps.cpp: $(PX4_BASE)/Tools/qurt_apps.py 
	$(PX4_BASE)/Tools/qurt_apps.py > $@

$(WORK_DIR)apps.o: $(WORK_DIR)apps.cpp
	$(call COMPILEXX,$<, $@)
	mv $(WORK_DIR)apps.cpp $(WORK_DIR)apps.cpp_sav

$(WORK_DIR)mainapp: $(WORK_DIR)apps.o $(PRODUCT_SHARED_LIB) 
	$(call LINK,$@, $^)

#
# Utility rules
#

.PHONY: clean
clean:			$(MODULE_CLEANS)
	@$(ECHO) %% cleaning
	$(Q) $(REMOVE) $(PRODUCT_ELF)
	$(Q) $(REMOVE) $(OBJS) $(DEP_INCLUDES) $(EXTRA_CLEANS)
