# ----------------------------------------------------------------------
# libpoff/Makefile
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
# Directories

PASCAL			= ${shell pwd}/..

include $(PASCAL)/Make.config
include $(PASCAL)/Make.defs

INCDIR			= $(PASCAL)/include
LIBDIR			= $(PASCAL)/lib

# ----------------------------------------------------------------------
# Objects and targets

LIBPOFFSRCS		= pfhandle.c pfproghandle.c pftprog.c \
			  pfsymhandle.c pftsymbol.c pofferr.c \
			  pfwhdr.c pfwrodata.c pfwsymbol.c pfwfname.c \
			  pfwprog.c pfwlineno.c pfwdbgfunc.c pfwreloc.c pfwstring.c \
			  pfwrite.c pfrhdr.c pfrsymbol.c pfrfname.c \
			  pfrprog.c pfrlineno.c pfrdbgfunc.c pfrrawlineno.c \
			  pfrrawreloc.c pfrstring.c pfread.c pfrseek.c \
			  pfrelease.c pfdbgcontainer.c pfdbgdiscard.c \
			  pfxprog.c pfxrodata.c pfiprog.c pfirodata.c \
			  pfdhdr.c pfdsymbol.c pfdreloc.c pflabel.c \
			  pflineno.c pfdbginfo.c pfswap.c
LIBPOFFOBJS		= $(LIBPOFFSRCS:.c=.o)

all: libpoff.a
.PHONY: all libpoff.a clean

$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(LIBDIR)/libpoff.a: $(LIBPOFFOBJS)
	$(RM) $@
	$(AR) $(ARFLAGS) $@ $^

libpoff.a: $(LIBDIR)/libpoff.a

clean:
	$(RM) libpoff.a *.o core *~

# ----------------------------------------------------------------------
