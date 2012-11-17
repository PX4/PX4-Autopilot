############################################################################
# fs/Makefile
#
#   Copyright (C) 2007, 2008, 2011-2012 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/Make.defs

ASRCS	= 
AOBJS	= $(ASRCS:.S=$(OBJEXT))

CSRCS	=

# If there are no file descriptors configured, then a small part of the
# logic in this directory may still apply to socket descriptors

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)

# Socket descriptor support

CSRCS	+= fs_close.c fs_read.c fs_write.c fs_ioctl.c fs_poll.c fs_select.c
endif

# Support for network access using streams

ifneq ($(CONFIG_NFILE_STREAMS),0)
CSRCS	+= fs_fdopen.c 
endif

else

# Common file/socket descriptor support

CSRCS	+= fs_close.c fs_closedir.c fs_dup.c fs_dup2.c fs_fcntl.c \
		   fs_filedup.c fs_filedup2.c fs_ioctl.c fs_lseek.c fs_open.c \
		   fs_opendir.c fs_poll.c fs_read.c fs_readdir.c fs_rewinddir.c \
		   fs_seekdir.c fs_stat.c fs_statfs.c fs_select.c fs_write.c
CSRCS	+= fs_files.c fs_foreachinode.c fs_inode.c fs_inodeaddref.c \
		   fs_inodefind.c fs_inoderelease.c fs_inoderemove.c \
		   fs_inodereserve.c
CSRCS	+= fs_registerdriver.c fs_unregisterdriver.c
CSRCS	+= fs_registerblockdriver.c fs_unregisterblockdriver.c \
		   fs_findblockdriver.c fs_openblockdriver.c fs_closeblockdriver.c

include mmap/Make.defs

# Stream support

ifneq ($(CONFIG_NFILE_STREAMS),0)
CSRCS	+= fs_fdopen.c 
endif

# System logging to a character device (or file)

ifeq ($(CONFIG_SYSLOG),y)
ifeq ($(CONFIG_SYSLOG_CHAR),y)
CSRCS	+= fs_syslog.c 
endif
endif

# Additional files required is mount-able file systems are supported

ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)
CSRCS	+= fs_fsync.c fs_mkdir.c fs_mount.c fs_rename.c fs_rmdir.c \
		   fs_umount.c fs_unlink.c
CSRCS	+= fs_foreachmountpoint.c 
include fat/Make.defs
include romfs/Make.defs
include nxffs/Make.defs
include nfs/Make.defs
endif
endif

COBJS	= $(CSRCS:.c=$(OBJEXT))

SRCS	= $(ASRCS) $(CSRCS)
OBJS	= $(AOBJS) $(COBJS)

BIN		= libfs$(LIBEXT)

SUBDIRS	= mmap fat romfs nxffs:nfs
VPATH	= mmap:fat:romfs:nxffs:nfs

all:	$(BIN)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) --dep-path . $(MMAPDEPPATH) $(FATDEPPATH) $(ROMFSDEPPATH) $(NXFFSDEPPATH) $(NFSDEPPATH) \
		"$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE Make.dep)
	$(call DELFILE .depend)

-include Make.dep
