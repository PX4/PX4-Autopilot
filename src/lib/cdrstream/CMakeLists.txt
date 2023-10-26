############################################################################
#
#   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

if(CONFIG_LIB_CDRSTREAM)
	set(BUILD_SHARED_LIBS NO)

	# CycloneDDS CDR serializer
	include(${CMAKE_CURRENT_LIST_DIR}/cyclonedds/src/core/cdr/CMakeLists.txt)
	target_compile_options(cdr PUBLIC
			       -Wno-cast-align
			       -Wno-double-promotion
			       $<$<COMPILE_LANGUAGE:C>:-Wno-implicit-function-declaration -Wno-nested-externs>
			       -DNDEBUG)
	px4_add_git_submodule(TARGET git_cyclonedds PATH "cyclonedds")
	px4_add_git_submodule(TARGET git_rosidl PATH "rosidl")
	add_dependencies(cdr git_cyclonedds git_rosidl uorb_headers parameters px4_platform)
	target_sources(cdr PRIVATE dds_serializer.c)
	target_include_directories(cdr PUBLIC ${CMAKE_CURRENT_LIST_DIR})
endif()
