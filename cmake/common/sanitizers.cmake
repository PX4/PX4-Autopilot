############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

option(SANITIZE_ADDRESS "Enable AddressSanitizer" Off)
option(SANITIZE_MEMORY "Enable MemorySanitizer" Off)
option(SANITIZE_THREAD "Enable ThreadSanitizer" Off)
option(SANITIZE_UNDEFINED "Enable UndefinedBehaviorSanitizer" Off)

if (SANITIZE_ADDRESS)
        message(STATUS "address sanitizer enabled")

        # environment variables
        #  ASAN_OPTIONS=detect_stack_use_after_return=1
        #  ASAN_OPTIONS=check_initialization_order=1
        add_compile_options(
                -g3
                -fno-omit-frame-pointer
                -fsanitize=address
                #-fsanitize-address-use-after-scope
        )

elseif(SANITIZE_MEMORY)
        message(STATUS "thread sanitizer enabled")

        add_compile_options(
                -g3
                -fsanitize=memory
        )

elseif(SANITIZE_THREAD)
        message(STATUS "thread sanitizer enabled")

        add_compile_options(
                -g3
                -fsanitize=thread
        )

elseif(SANITIZE_UNDEFINED)
        message(STATUS "undefined behaviour sanitizer enabled")

        add_compile_options(
                -g3
                #-fsanitize=alignment
                -fsanitize=bool
                -fsanitize=bounds
                -fsanitize=enum
                #-fsanitize=float-cast-overflow
                -fsanitize=float-divide-by-zero
                #-fsanitize=function
                -fsanitize=integer-divide-by-zero
                -fsanitize=nonnull-attribute
                -fsanitize=null
                -fsanitize=object-size
                -fsanitize=return
                -fsanitize=returns-nonnull-attribute
                -fsanitize=shift
                -fsanitize=signed-integer-overflow
                -fsanitize=unreachable
                #-fsanitize=unsigned-integer-overflow
                -fsanitize=vla-bound
                -fsanitize=vptr
        )

endif()
