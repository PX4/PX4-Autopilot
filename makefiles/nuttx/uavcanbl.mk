#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
#	The extention uesd on uavcan bootloadable binary images
#
UAVCAN_BL_EXT=uavcan.bin

#
#	The tool to embed the uavcan bootloader application descriptor
#
MKUAVCANBL	 	= $(PX4_TOOLS_DIR)make_can_boot_descriptor.py

#
# Get a short version string provided by git
# This assumes that git command is available and that
# the directory holding this file also contains .git directory
#
MKUAVCANBL_GIT_DESC := $(shell git rev-list HEAD --max-count=1 --abbrev=8 --abbrev-commit)
ifneq ($(words $(MKUAVCANBL_GIT_DESC)),1)
    MKUAVCANBL_GIT_DESC := ffffffff
endif
export MKUAVCANBL_GIT_DESC


MKUAVCANBLNAME=$1-$(UAVCANBLID_HW_VERSION_MAJOR).$(UAVCANBLID_HW_VERSION_MINOR)-$(UAVCANBLID_SW_VERSION_MAJOR).$(UAVCANBLID_SW_VERSION_MINOR).$(MKUAVCANBL_GIT_DESC)




