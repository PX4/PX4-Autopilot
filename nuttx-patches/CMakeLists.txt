############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
############################################################################

set(nuttx_patches
	00001-REJECTED-add-math.h.patch
	00002-REJECTED-fix-shadow-wanings.patch
	00003-REJECTED-add-var-expansion-in-nsh-parse.patch
	00004-REJECTED-avoid-export-copy-with-export-insitu.patch
	00005-REJECTED-support-c++11.patch
	00006-REJECTED-cstdint-fix.patch
	00007-REJECTED-silence-jobserver-warnings.patch
	00008-REJECTED-static-assert-fix.patch
	00009-REJECTED-ctype-fix-shadow-wanings.patch
	00010-BACKPORT-stm32-flash-F4-dcache-corruption-fix-no-HSI-on.patch
	00011-BACKPORT-priority-restoration-fix.patch
	00012-BACKPORT-stack-coloration-overreach-fix.patch
	00013-BACKPORT-stm32-serial-dma-hotfix.patch
	00014-BACKPORT-i2c-hotfix.patch
	00015-BACKPORT-stm32f7-DTCM.patch
	00016-BACKPORT-fix-CRTSCTS-defines.patch
	00017-BACKPORT-cdcacm.patch
	00018-BACKPORT-stm32-serial-break.patch
	00019-BACKPORT-stm32-rcc-keep-HSI-on.patch
	00020-BACKPORT-ramtron-CONFIG-prefix.patch
	00021-BACKPORT-stm32f3x-add-BKP.patch
	00022-BACKPORT-stm32-bkp-reference-fix.patch
	00023-BACKPORT-stm32f7-bkp-reference-fix.patch
	00024-BACKPORT-stm32f7-serial-dma-hotfix.patch
	00025-BACKPORT-add-set-ex-to-nsh.patch
        00026-BACKPORT-stm32-config-serial-DMA-buf-size.patch
	90000-PENDING-wip-inflight-to-upstream.patch
	)

# add absolute path and sort
set(px4_nuttx_patches)
foreach(patch_file ${nuttx_patches})
	list(APPEND px4_nuttx_patches ${CMAKE_CURRENT_SOURCE_DIR}/${patch_file})
endforeach()
list(SORT px4_nuttx_patches)
set(px4_nuttx_patches ${px4_nuttx_patches} PARENT_SCOPE)

file(GLOB patches_found *.patch)
foreach(patch_file ${patches_found})
	list (FIND px4_nuttx_patches ${patch_file} _index)
	if (${_index} EQUAL -1)
		message(WARNING "${patch_file} missing from nuttx-patches/CMakeLists.txt")
	endif()
endforeach()
