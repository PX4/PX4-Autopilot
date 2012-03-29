############################################################################
# apps/examples/Makefile
#
#   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/.config	# Current configuration

# Sub-directories

SUBDIRS = adc buttons can cdcacm composite dhcpd ftpc ftpd hello helloxx \
	hidkbd igmp lcdrw mm mount nettest nsh null nx nxconsole nxffs nxflat \
	nxhello nximage nxlines nxtext ostest pashello pipe poll pwm qencoder \
	rgmp romfs serloop telnetd thttpd tiff touchscreen udp uip usbserial \
	sendmail usbstorage usbterm wget wlan

# Sub-directories that might need context setup.  Directories may need
# context setup for a variety of reasons, but the most common is because
# the example may be built as an NSH built-in function.
#
# Directories that may be built as NSH built-in functions may have their
# own configuration setting (like CONFIG_EXAMPLES_HELLOXX_BUILTIN), but
# many only depend on the generic CONFIG_NSH_BUILTIN_APPS setting.  And
# there a few which an ONLY be built as NSH built-in applications; these
# are included in the list unconditionally.

CNTXTDIRS = pwm

ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
CNTXTDIRS += adc can cdcacm composite ftpd dhcpd nettest qencoder telnetd
endif

ifeq ($(CONFIG_EXAMPLES_HELLO_BUILTIN),y)
CNTXTDIRS += hello
endif
ifeq ($(CONFIG_EXAMPLES_HELLOXX_BUILTIN),y)
CNTXTDIRS += helloxx
endif
ifeq ($(CONFIG_EXAMPLES_LCDRW_BUILTIN),y)
CNTXTDIRS += lcdrw
endif
ifeq ($(CONFIG_EXAMPLES_NX_BUILTIN),y)
CNTXTDIRS += nx
endif
ifeq ($(CONFIG_EXAMPLES_NXHELLO_BUILTIN),y)
CNTXTDIRS += nxhello
endif
ifeq ($(CONFIG_EXAMPLES_NXIMAGE_BUILTIN),y)
CNTXTDIRS += nximage
endif
ifeq ($(CONFIG_EXAMPLES_LINES_BUILTIN),y)
CNTXTDIRS += nxlines
endif
ifeq ($(CONFIG_EXAMPLES_NXTEXT_BUILTIN),y)
CNTXTDIRS += nxtext
endif
ifeq ($(CONFIG_EXAMPLES_OSTEST_BUILTIN),y)
CNTXTDIRS += ostest
endif
ifeq ($(CONFIG_EXAMPLES_TIFF_BUILTIN),y)
CNTXTDIRS += tiff
endif
ifeq ($(CONFIG_EXAMPLES_TOUCHSCREEN_BUILTIN),y)
CNTXTDIRS += touchscreen
endif
ifeq ($(CONFIG_EXAMPLES_USBMSC_BUILTIN),y)
CNTXTDIRS += usbstorage
endif
ifeq ($(CONFIG_EXAMPLES_USBTERM_BUILTIN),y)
CNTXTDIRS += usbterm
endif

all: nothing

.PHONY: nothing context depend clean distclean

nothing:

context:
	@for dir in $(CNTXTDIRS) ; do \
		$(MAKE) -C $$dir context TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"; \
	done

depend:
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir depend TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"; \
	done

clean:
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir clean TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"; \
	done

distclean: clean
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir distclean TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"; \
	done

-include Make.dep

