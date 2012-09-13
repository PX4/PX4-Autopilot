############################################################################
# toolchain/nxflat/nxflat.mk
#
#   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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
# 3. Neither the name NuttX nor the names of its contributors may be
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
############################################################################

NXFLAT_DIR	= $(TOPDIR)/toolchain/nxflat

$(NXFLAT_DIR)/.compiled:
ifeq ($(strip $(BR2_GCC_TARGET_ARCH)),"armv7-m")
	echo "NUTTX_DIR: $(NUTTX_DIR)"
	$(MAKE) -C $(NXFLAT_DIR) BINUTILS_DIR="$(BINUTILS_DIR)" \
		BINUTILS_DIR1="$(BINUTILS_DIR1)" ARCH=thumb2 CC="$(HOSTCC)"
else
	echo "NUTTX_DIR: $(NUTTX_DIR)"
	$(MAKE) -C $(NXFLAT_DIR) BINUTILS_DIR="$(BINUTILS_DIR)"  \
		BINUTILS_DIR1="$(BINUTILS_DIR1)" ARCH=$(BR2_ARCH) CC="$(HOSTCC)"
endif
	touch $@

$(NXFLAT_DIR)/.installed: $(NXFLAT_DIR)/.compiled
	install -m 755 $(NXFLAT_DIR)/mknxflat $(STAGING_DIR)/bin/mknxflat
	install -m 755 $(NXFLAT_DIR)/ldnxflat $(STAGING_DIR)/bin/ldnxflat
	install -m 755 $(NXFLAT_DIR)/readnxflat $(STAGING_DIR)/bin/readnxflat
	touch $@

nxflat: binutils $(NXFLAT_DIR)/.installed

nxflat-clean:
	$(MAKE) -C $(NXFLAT_DIR) clean
	rm -f $(STAGING_DIR)/bin/mknxflat $(STAGING_DIR)/bin/ldnxflat $(STAGING_DIR)/bin/readnxflat
	rm -f $(NXFLAT_DIR)/.compiled $(NXFLAT_DIR)/.installed

nxflat-dirclean: nxflat-clean
	true

ifeq ($(strip $(BR2_PACKAGE_NXFLAT)),y)
TARGETS += nxflat
endif

