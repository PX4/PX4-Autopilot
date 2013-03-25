#
# Makefile to generate a PX4FMU ROMFS image.
#
# In normal use, 'make install' will generate a new ROMFS header and place it
# into the px4fmu configuration in the appropriate location.
#

#
# Directories of interest
#
SRCROOT		?= $(dir $(lastword $(MAKEFILE_LIST)))
BUILDROOT	?= $(SRCROOT)/img
ROMFS_HEADER	?= $(SRCROOT)/../nuttx/configs/px4fmu/include/nsh_romfsimg.h

#
# List of files to install in the ROMFS, specified as <source>~<destination>
#
ROMFS_FSSPEC	:= $(SRCROOT)/scripts/rcS~init.d/rcS \
		   $(SRCROOT)/scripts/rc.sensors~init.d/rc.sensors \
		   $(SRCROOT)/scripts/rc.logging~init.d/rc.logging \
		   $(SRCROOT)/scripts/rc.standalone~init.d/rc.standalone \
		   $(SRCROOT)/scripts/rc.PX4IO~init.d/rc.PX4IO \
		   $(SRCROOT)/scripts/rc.PX4IOAR~init.d/rc.PX4IOAR \
		   $(SRCROOT)/scripts/rc.FMU_quad_x~init.d/rc.FMU_quad_x \
		   $(SRCROOT)/scripts/rc.usb~init.d/rc.usb \
		   $(SRCROOT)/scripts/rc.hil~init.d/rc.hil \
		   $(SRCROOT)/mixers/FMU_pass.mix~mixers/FMU_pass.mix \
		   $(SRCROOT)/mixers/FMU_Q.mix~mixers/FMU_Q.mix \
		   $(SRCROOT)/mixers/FMU_X5.mix~mixers/FMU_X5.mix \
		   $(SRCROOT)/mixers/FMU_AERT.mix~mixers/FMU_AERT.mix \
		   $(SRCROOT)/mixers/FMU_AET.mix~mixers/FMU_AET.mix \
		   $(SRCROOT)/mixers/FMU_RET.mix~mixers/FMU_ERT.mix \
		   $(SRCROOT)/mixers/FMU_quad_x.mix~mixers/FMU_quad_x.mix \
		   $(SRCROOT)/mixers/FMU_quad_+.mix~mixers/FMU_quad_+.mix \
		   $(SRCROOT)/mixers/FMU_hex_x.mix~mixers/FMU_hex_x.mix \
		   $(SRCROOT)/mixers/FMU_hex_+.mix~mixers/FMU_hex_+.mix \
		   $(SRCROOT)/mixers/FMU_octo_x.mix~mixers/FMU_octo_x.mix \
		   $(SRCROOT)/mixers/FMU_octo_+.mix~mixers/FMU_octo_+.mix \
		   $(SRCROOT)/logging/logconv.m~logging/logconv.m

# the EXTERNAL_SCRIPTS variable is used to add out of tree scripts
# to ROMFS. 
ROMFS_FSSPEC += $(EXTERNAL_SCRIPTS)

#
# Add the PX4IO firmware to the spec if someone has dropped it into the
# source directory, or otherwise specified its location.
#
# Normally this is only something you'd do when working on PX4IO; most
# users will upgrade with firmware off the microSD card.
#
PX4IO_FIRMWARE	?= $(SRCROOT)/px4io.bin
ifneq ($(wildcard $(PX4IO_FIRMWARE)),)
ROMFS_FSSPEC	+=  $(PX4IO_FIRMWARE)~px4io.bin
endif

################################################################################
# No user-serviceable parts below
################################################################################

#
# Just the source files from the ROMFS spec, so that we can fail cleanly if they don't
# exist
#
ROMFS_SRCFILES	 = $(foreach spec,$(ROMFS_FSSPEC),$(firstword $(subst ~, ,$(spec))))

#
# Just the destination directories from the ROMFS spec
#
ROMFS_DIRS	 = $(sort $(dir $(foreach spec,$(ROMFS_FSSPEC),$(lastword $(subst ~, ,$(spec))))))


#
# Intermediate products
#
ROMFS_IMG	 = $(BUILDROOT)/romfs.img
ROMFS_WORKDIR	 = $(BUILDROOT)/romfs

#
# Convenience target for rebuilding the ROMFS header
#
all:		$(ROMFS_HEADER)

$(ROMFS_HEADER): $(ROMFS_IMG) $(dir $(ROMFS_HEADER))
	@echo Generating the ROMFS header...
	@(cd $(dir $(ROMFS_IMG)) && xxd -i $(notdir $(ROMFS_IMG))) | sed -e 's/char/const char/' > $@

$(ROMFS_IMG):	$(ROMFS_WORKDIR)
	@echo Generating the ROMFS image...
	@genromfs -f $@ -d $(ROMFS_WORKDIR) -V "NSHInitVol"

$(ROMFS_WORKDIR): $(ROMFS_SRCFILES)
	@echo Rebuilding the ROMFS work area...
	@rm -rf $(ROMFS_WORKDIR)
	@mkdir -p $(ROMFS_WORKDIR)
	@for dir in $(ROMFS_DIRS) ; do mkdir -p $(ROMFS_WORKDIR)/$$dir; done
	@for spec in $(ROMFS_FSSPEC) ; do \
		echo $$spec | sed -e 's%^.*~%  %' ;\
		`echo "cp $$spec" | sed -e 's%~% $(ROMFS_WORKDIR)/%'` ;\
	 done

$(BUILDROOT):
	@mkdir -p $(BUILDROOT)

clean:
	@rm -rf $(BUILDROOT)

distclean:	clean
	@rm -f $(PX4IO_FIRMWARE) $(ROMFS_HEADER)

.PHONY:	all install clean distclean

#
# Hacks and fixups
#
SYSTYPE			 = $(shell uname)

ifeq ($(SYSTYPE),Darwin)
# PATH inherited by Eclipse may not include toolchain install location 
export PATH			 := $(PATH):/usr/local/bin
endif

