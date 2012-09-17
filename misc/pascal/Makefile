# ----------------------------------------------------------------------
# Makefile
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
# Directories

PASCAL			= ${shell pwd}

include $(PASCAL)/Make.config
include $(PASCAL)/Make.defs

INCDIR			= $(PASCAL)/include
LIBDIR			= $(PASCAL)/lib
BINDIR-$(CONFIG_INSN16)	= $(PASCAL)/bin16
BINDIR-$(CONFIG_INSN32)	= $(PASCAL)/bin32
LIBPOFFDIR		= $(PASCAL)/libpoff
LIBPASDIR		= $(PASCAL)/libpas
PASDIR			= $(PASCAL)/pascal
PLINKDIR		= $(PASCAL)/plink
TESTDIR			= $(PASCAL)/tests
INSN-$(CONFIG_INSN16)	= $(PASCAL)/insn16
INSN-$(CONFIG_INSN32)	= $(PASCAL)/insn32
LIBINSNDIR		= $(INSN-y)/libinsn

# ----------------------------------------------------------------------
# Objects and targets

LIBS			= $(LIBDIR)/libpoff.a $(LIBDIR)/libpas.a \
			  $(LIBDIR)/libinsn.a

all: pascal popt regm plink plist prun
.PHONY: all config.h libpoff.a libpas.a libinsn.a pascal popt regm plink plist prun clean deep-clean

$(INCDIR)/config.h: Make.config
	@$(MAKE) -f Make.config.h

config.h: $(INCDIR)/config.h

$(LIBDIR):
	mkdir $(LIBDIR)

$(LIBDIR)/libpoff.a: $(LIBDIR) config.h
	@$(MAKE) -C $(LIBPOFFDIR) libpoff.a

libpoff.a: $(LIBDIR)/libpoff.a

$(LIBDIR)/libpas.a: $(LIBDIR) config.h
	@$(MAKE) -C $(LIBPASDIR) libpas.a

libpas.a: $(LIBDIR)/libpas.a

$(LIBDIR)/libinsn.a: $(LIBDIR) config.h
	@$(MAKE) -C $(LIBINSNDIR) libinsn.a

libinsn.a: $(LIBDIR)/libinsn.a

$(BINDIR-y):
	mkdir $(BINDIR-y)

$(BINDIR-y)/pascal: $(BINDIR-y) config.h $(LIBS)
	@$(MAKE) -C $(PASDIR)

pascal: $(BINDIR-y)/pascal

$(BINDIR-y)/popt: $(BINDIR-y) config.h $(LIBS)
	@$(MAKE) -C $(INSN-y) popt

popt: $(BINDIR-y)/popt

$(BINDIR-y)/regm: $(BINDIR-y) config.h $(LIBS)
ifeq ($(CONFIG_REGM),y)
	@$(MAKE) -C $(INSN-y) regm
endif

regm: $(BINDIR-y)/regm

$(BINDIR-y)/plink: $(BINDIR-y) config.h $(LIBS)
	@$(MAKE) -C $(PLINKDIR)

plink: $(BINDIR-y)/plink

$(BINDIR-y)/prun: $(BINDIR-y) config.h $(LIBS)
	@$(MAKE) -C $(INSN-y) prun

prun: $(BINDIR-y)/prun

$(BINDIR-y)/plist: $(BINDIR-y) config.h $(LIBS)
	@$(MAKE) -C $(INSN-y) plist

plist: $(BINDIR-y)/plist

clean:
	$(RM) -f core *~
	$(RM) -rf $(LIBDIR)
	$(RM) -rf bin16 bin32
	$(MAKE) -f Make.config.h clean
	$(MAKE) -C $(LIBPOFFDIR) clean
	$(MAKE) -C $(LIBPASDIR) clean
	$(MAKE) -C $(PASDIR) clean
	$(MAKE) -C $(PLINKDIR) clean
	$(MAKE) -C $(INSN-y) clean
	find . -name \*~ -exec rm -f {} \;
	find tests -name "*.err" -exec rm -f {} \;
	find tests -name "*.lst" -exec rm -f {} \;
	find tests -name "*.pex" -exec rm -f {} \;
	find tests -name "*.o1" -exec rm -f {} \;
	find tests -name "*.o" -exec rm -f {} \;

deep-clean: clean
	rm -f .config include/config.h Make.config
	$(RM) bin16/*
	$(RM) bin32/*

# ----------------------------------------------------------------------
