############################################################################
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
############################################################################

#
# System utility library
#

CSRCS		 = err.c \
		   hx_stream.c \
		   perf_counter.c \
		   param/param.c \
		   bson/tinybson.c \
		   conversions.c \
		   cpuload.c \
		   getopt_long.c \
		   up_cxxinitialize.c \
		   control/test_params.c

#		   ppm_decode.c \


CXXSRCS   =  test/test.cpp \
			 control/block/Block.cpp \
			 control/block/BlockParam.cpp \
			 control/block/UOrbPublication.cpp \
			 control/block/UOrbSubscription.cpp \
			 control/blocks.cpp \
			 math/Vector.cpp \
			 math/Matrix.cpp \
			 math/Kalman.cpp

CXXHDRS   =  test/test.hpp \
			 control/block/Block.hpp \
			 control/block/BlockParam.hpp \
			 control/block/UOrbPublication.hpp \
			 control/block/UOrbSubscription.hpp \
			 control/blocks.hpp \
			 control/fixedwing.hpp \
			 math/Vector.hpp \
			 math/Matrix.hpp \
			 math/Kalman.hpp

#
# XXX this really should be a CONFIG_* test
#
ifeq ($(TARGET),px4fmu)
CSRCS		+= systemlib.c \
		pid/pid.c \
		geo/geo.c
CXXSRCS     += math/arm/Vector.cpp \
		math/arm/Matrix.cpp
CXXHDRS     += math/arm/Vector.hpp \
	    math/arm/Matrix.hpp
else
CXXSRCS     += math/generic/Vector.cpp \
		math/generic/Matrix.cpp
CXXHDRS     += math/generic/Vector.hpp \
	    math/generic/Matrix.hpp
endif

include $(APPDIR)/mk/app.mk
