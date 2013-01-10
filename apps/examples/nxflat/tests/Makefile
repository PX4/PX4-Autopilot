############################################################################
# apps/examples/nxflat/tests/Makefile
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

# Most of these do no build yet
#SUBDIRS	= errno hello hello++ longjmp mutex pthread signal task struct
SUBDIRS		= errno hello mutex pthread task struct

NXFLAT_DIR	= $(APPDIR)/examples/nxflat
TESTS_DIR	= $(NXFLAT_DIR)/tests
ROMFS_DIR	= $(TESTS_DIR)/romfs
ROMFS_IMG	= $(TESTS_DIR)/romfs.img
ROMFS_HDR	= $(TESTS_DIR)/romfs.h
ROMFS_DIRLIST	= $(TESTS_DIR)/dirlist.h
SYMTAB		= $(TESTS_DIR)/symtab.h

define DIR_template
$(1)_$(2):
	@$(MAKE) -C $(1) $(3) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" ROMFS_DIR="$(ROMFS_DIR)" CROSSDEV=$(CROSSDEV)
endef

all: $(ROMFS_HDR) $(ROMFS_DIRLIST) $(SYMTAB)
.PHONY: all build clean install populate

$(foreach DIR, $(SUBDIRS), $(eval $(call DIR_template,$(DIR),build, all)))
$(foreach DIR, $(SUBDIRS), $(eval $(call DIR_template,$(DIR),clean,clean)))
$(foreach DIR, $(SUBDIRS), $(eval $(call DIR_template,$(DIR),install,install)))

# Build program(s) in each sud-directory 

build: $(foreach DIR, $(SUBDIRS), $(DIR)_build)

# Install each program in the romfs directory

install: $(foreach DIR, $(SUBDIRS), $(DIR)_install)

# Create the romfs directory

$(ROMFS_DIR):
	@mkdir $(ROMFS_DIR)

# Populate the romfs directory

populate:  $(ROMFS_DIR) build install

# Create the romfs.img file from the populated romfs directory

$(ROMFS_IMG): populate
	@genromfs -f $@ -d $(ROMFS_DIR) -V "NXFLATTEST"

# Create the romfs.h header file from the romfs.img file

$(ROMFS_HDR) : $(ROMFS_IMG)
	@(cd $(TESTS_DIR); xxd -i romfs.img | sed -e "s/^unsigned/static const unsigned/g" >$@)

# Create the dirlist.h header file from the romfs directory 

$(ROMFS_DIRLIST) : populate
	@$(TESTS_DIR)/mkdirlist.sh $(ROMFS_DIR) >$@

# Create the exported symbol table list from the derived *-thunk.S files

$(SYMTAB): build
	@$(TESTS_DIR)/mksymtab.sh $(TESTS_DIR) >$@

# Clean each subdirectory

clean: $(foreach DIR, $(SUBDIRS), $(DIR)_clean)
	@rm -f $(ROMFS_HDR) $(ROMFS_DIRLIST) $(ROMFS_IMG) $(SYMTAB)
	@rm -rf $(ROMFS_DIR)


