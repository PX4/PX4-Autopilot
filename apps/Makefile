############################################################################
# apps/Makefile
#
#   Copyright (C) 2011-2012 Uros Platise. All rights reserved.
#   Authors: Uros Platise <uros.platise@isotel.eu>
#            Gregory Nutt <gnutt@nuttx.org>
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

-include $(TOPDIR)/Make.defs
-include $(TOPDIR)/.config

APPDIR = ${shell pwd}

# Application Directories

# CONFIGURED_APPS is the list of all configured built-in directories/built
#   action. It is created by the configured appconfig file (a copy of which
#   appears in this directory as .config)
# SUBDIRS is the list of all directories containing Makefiles.  It is used
#   only for cleaning. namedapp must always be the first in the list.  This
#   list can be extended by the .config file as well

CONFIGURED_APPS =
SUBDIRS = examples graphics interpreters modbus namedapp nshlib netutils system vsn

# There are two different mechanisms for obtaining the list of configured
# directories:
#
# (1) In the legacy method, these paths are all provided in the appconfig
#     file that is copied to the top-level apps/ directory as .config
# (2) With the development of the NuttX configuration tool, however, the
#     selected applications are now enabled by the configuration tool.
#     The apps/.config file is no longer used.  Instead, the set of
#     configured build directories can be found by including a Make.defs
#     file contained in each of the apps/subdirectories.
#
# When the NuttX configuration tools executes, it will always define the
# configure CONFIG_NUTTX_NEWCONFIG to select between these two cases.  Then
# legacy appconfig files will still work but newly configuration files will
# also work.  Eventually the CONFIG_NUTTX_NEWCONFIG option will be phased
# out.

ifeq ($(CONFIG_NUTTX_NEWCONFIG),y)

include examples/Make.defs
include graphics/Make.defs
include interpreters/Make.defs
include modbus/Make.defs
include namedapp/Make.defs
include netutils/Make.defs
include nshlib/Make.defs
include system/Make.defs
include vsn/Make.defs

# INSTALLED_APPS is the list of currently available application directories.  It
# is the same as CONFIGURED_APPS, but filtered to exclude any non-existent
# application directory. namedapp is always in the list of applications to be
# built.

INSTALLED_APPS =

# The legacy case:

else
-include .config

# INSTALLED_APPS is the list of currently available application directories.  It
# is the same as CONFIGURED_APPS, but filtered to exclude any non-existent
# application directory. namedapp is always in the list of applications to be
# built.

INSTALLED_APPS = namedapp
endif

# Create the list of available applications (INSTALLED_APPS)

define ADD_BUILTIN
INSTALLED_APPS += ${shell if [ -r $1/Makefile ]; then echo "$1"; fi}
endef

$(foreach BUILTIN, $(CONFIGURED_APPS), $(eval $(call ADD_BUILTIN,$(BUILTIN))))

# The external/ directory may also be added to the INSTALLED_APPS.  But there
# is no external/ directory in the repository.  Rather, this directory may be
# provided by the user (possibly as a symbolic link) to add libraries and
# applications to the standard build from the repository.

INSTALLED_APPS += ${shell if [ -r external/Makefile ]; then echo "external"; fi}
SUBDIRS += ${shell if [ -r external/Makefile ]; then echo "external"; fi}

# The final build target

BIN = libapps$(LIBEXT)

# Build targets

all: $(BIN)
.PHONY: $(INSTALLED_APPS) context depend clean distclean

$(INSTALLED_APPS):
	@$(MAKE) -C $@ TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)";

$(BIN):	$(INSTALLED_APPS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

.context:
	@for dir in $(INSTALLED_APPS) ; do \
		rm -f $$dir/.context ; \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)"  APPDIR="$(APPDIR)" context ; \
	done
	@touch $@

context: .context

.depend: context Makefile $(SRCS)
	@for dir in $(INSTALLED_APPS) ; do \
		rm -f $$dir/.depend ; \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)"  APPDIR="$(APPDIR)" depend ; \
	done
	@touch $@

depend: .depend

clean:
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir clean TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"; \
	done
	@rm -f $(BIN) *~ .*.swp *.o
	$(call CLEAN)

distclean: # clean
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir distclean TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"; \
	done
	@rm -f .config .context .depend
	@( if [ -e external ]; then \
		echo "********************************************************"; \
		echo "* The external directory/link must be removed manually *"; \
		echo "********************************************************"; \
	   fi; \
	) 


