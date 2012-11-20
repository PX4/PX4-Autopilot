############################################################################
# apps/examples/thttpd/content/Makefile
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

SUBDIRS		= hello tasks netstat
INSTALL_FILES	= index.html style.css

THTTPD_DIR	= $(APPDIR)/examples/thttpd
CONTENT_DIR	= $(THTTPD_DIR)/content
ROMFS_DIR	= $(CONTENT_DIR)/romfs
ROMFS_IMG	= $(CONTENT_DIR)/romfs.img
ROMFS_HDR	= $(CONTENT_DIR)/romfs.h
ROMFSCGI_DIR	= $(ROMFS_DIR)/cgi-bin
SYMTAB		= $(CONTENT_DIR)/symtab.h

define DIR_template
$(1)_$(2):
	@$(MAKE) -C $(1) $(3) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) CGI_DIR="$(ROMFSCGI_DIR)"
endef

all: $(ROMFS_HDR) $(SYMTAB)
.PHONY: all build clean install populate

$(foreach DIR, $(SUBDIRS), $(eval $(call DIR_template,$(DIR),build, all)))
$(foreach DIR, $(SUBDIRS), $(eval $(call DIR_template,$(DIR),clean,clean)))
$(foreach DIR, $(SUBDIRS), $(eval $(call DIR_template,$(DIR),install,install)))

# Build program(s) in each sud-directory 

build: $(foreach DIR, $(SUBDIRS), $(DIR)_build)

# Install each program in the romfs directory

install: $(foreach DIR, $(SUBDIRS), $(DIR)_install)
	@( for file in $(INSTALL_FILES); do\
		install -m 0644 -D $${file} $(ROMFS_DIR)/$${file}; \
	done; )

# Create the romfs directory

$(ROMFS_DIR):
	@mkdir -p $(ROMFS_DIR)

$(ROMFSCGI_DIR): $(ROMFS_DIR)
	@mkdir -p $(ROMFSCGI_DIR)

# Populate the romfs directory

populate:  $(ROMFSCGI_DIR) build install

# Create the romfs.img file from the populated romfs directory

$(ROMFS_IMG): populate
	@genromfs -f $@ -d $(ROMFS_DIR) -V "THTTPDTEST"

# Create the romfs.h header file from the romfs.img file

$(ROMFS_HDR) : $(ROMFS_IMG)
	@(cd $(CONTENT_DIR); xxd -i romfs.img | sed -e "s/^unsigned/static const unsigned/g" >$@)

# Create the exported symbol table list from the derived *-thunk.S files

$(SYMTAB): build
	@$(CONTENT_DIR)/mksymtab.sh $(CONTENT_DIR) >$@

# Clean each subdirectory

clean: $(foreach DIR, $(SUBDIRS), $(DIR)_clean)
	$(call DELFILE$(ROMFS_HDR))
	$(call DELFILE, $(ROMFS_IMG))
	$(call DELFILE, $(SYMTAB))
	@rm -rf $(ROMFS_DIR)
	$(call CLEAN)


