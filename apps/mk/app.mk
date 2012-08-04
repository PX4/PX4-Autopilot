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
# Common Makefile for nsh command modules and utility libraries; should be 
# included by the module-specific Makefile.
#
# To build an app that appears as an nsh external command, the caller 
# must define:
#
# APPNAME	- the name of the application, defaults to the name
#		  of the parent directory.
#
# If APPNAME is not defined, a utility library is built instead. The library
# name is normally automatically determined, but it can be overridden by
# setting:
#
# LIBNAME	- the name of the library, defaults to the name of the
#		  directory
#
# The calling makefile may also set:
#
# ASRCS		- list of assembly source files, defaults to all .S 
#		  files in the directory
#
# CSRCS		- list of C source files, defaults to all .c files
#		  in the directory
#
# CXXSRCS	- list of C++ source files, defaults to all .cpp
#		  files in the directory
#
# INCLUDES	- list of directories to be added to the include
#		  search path
#
# PRIORITY	- thread priority for the command (defaults to 
#		  SCHED_PRIORITY_DEFAULT)
#
# STACKSIZE	- stack size for the command (defaults to 
#		  CONFIG_PTHREAD_STACK_DEFAULT)
#
# Symbols in the module are private to the module unless deliberately exported
# using the __EXPORT tag.
#

############################################################################
# No user-serviceable parts below
############################################################################


############################################################################
# Work out who included us so we can report decent errors
#
THIS_MAKEFILE	:= $(lastword $(MAKEFILE_LIST))
PARENT_MAKEFILE	:= $(lastword $(filter-out $(THIS_MAKEFILE),$(MAKEFILE_LIST)))

############################################################################
# Get configuration
#
-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

############################################################################
# Sanity-check the information we've been given and set any defaults
#
SRCDIR		?= $(dir $(PARENT_MAKEFILE))
PRIORITY	?= SCHED_PRIORITY_DEFAULT
STACKSIZE	?= CONFIG_PTHREAD_STACK_DEFAULT

INCLUDES	+= $(APPDIR)

ASRCS		?= $(wildcard $(SRCDIR)/*.S)
CSRCS		?= $(wildcard $(SRCDIR)/*.c)
CXXSRCS		?= $(wildcard $(SRCDIR)/*.cpp)

# if APPNAME is not set, this is a library
ifeq ($(APPNAME),)
LIBNAME		?= $(lastword $(subst /, ,$(realpath $(SRCDIR))))
endif

# there has to be a source file
ifeq ($(ASRCS)$(CSRCS)$(CXXSRCS),)
$(error $(realpath $(PARENT_MAKEFILE)): at least one of ASRCS, CSRCS or CXXSRCS must be set)
endif

# check that C++ is configured if we have C++ source files and we are building
ifneq ($(CXXSRCS),)
ifneq ($(CONFIG_HAVE_CXX),y)
ifeq ($(MAKECMDGOALS),build)
$(error $(realpath $(PARENT_MAKEFILE)): cannot set CXXSRCS if CONFIG_HAVE_CXX not set in configuration)
endif
endif
endif

############################################################################
# Adjust compilation flags to implement EXPORT
#
CFLAGS		+= -fvisibility=hidden -include $(APPDIR)/systemlib/visibility.h
CXXFLAGS	+= -fvisibility=hidden -include $(APPDIR)/systemlib/visibility.h

############################################################################
# Add extra include directories
#
CFLAGS		+= $(addprefix -I,$(INCLUDES))
CXXFLAGS	+= $(addprefix -I,$(INCLUDES))

############################################################################
# Things we are going to build
#

SRCS		 = $(ASRCS) $(CSRCS) $(CXXSRCS)
AOBJS		 = $(patsubst %.S,%.o,$(ASRCS))
COBJS		 = $(patsubst %.c,%.o,$(CSRCS))
CXXOBJS		 = $(patsubst %.cpp,%.o,$(CXXSRCS))
OBJS		 = $(AOBJS) $(COBJS) $(CXXOBJS)

# The prelinked object that we are ultimately going to build
ifneq ($(APPNAME),)
PRELINKOBJ	 = $(APPNAME).pre.o
else
PRELINKOBJ	 = $(LIBNAME).pre.o
endif

# The archive that the object file will be placed in
# XXX does WINTOOL ever get set?
ifeq ($(WINTOOL),y)
  INCDIROPT	= -w
  BIN		 = "$(shell cygpath -w  $(APPDIR)/libapps$(LIBEXT))"
else
  BIN		 = "$(APPDIR)/libapps$(LIBEXT)"
endif

############################################################################
# Rules for building things
#

all:		.built
.PHONY:		clean depend distclean

#
# Top-level build; add prelinked object to the apps archive
#
.built:		$(PRELINKOBJ)
	@$(call ARCHIVE, $(BIN), $(PRELINKOBJ))
	@touch $@

#
# Source dependencies
#
depend:		.depend
.depend:	$(MAKEFILE_LIST) $(SRCS)
	@$(MKDEP) --dep-path . $(CC) -- $(CFLAGS) -- $(CSRCS) >Make.dep
	@$(MKDEP) --dep-path . $(CXX) -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
	@touch $@

ifneq ($(APPNAME),)
#
# App registration
#
context:	.context
.context:	$(MAKEFILE_LIST)
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)
	@touch $@
else
context:
endif

#
# Object files
#
$(PRELINKOBJ):	$(OBJS)
	$(call PRELINK, $@, $(OBJS))

$(AOBJS): %.o : %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %.o : %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): %.o : %.cpp
	$(call COMPILEXX, $<, $@)

#
# Tidying up
#
clean:
	@rm -f $(OBJS) $(PRELINKOBJ) Make.dep .built
	$(call CLEAN)

distclean:	clean
	@rm -f Make.dep .depend

-include Make.dep
