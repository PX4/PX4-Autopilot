AS		= /usr/local/bin/as-z80
ASFLAGS		= -xlosp

CPP		= /usr/local/bin/sdcpp
CPPFLAGS	= -D__ASSEMBLY__

CC		= /usr/local/bin/sdcc
CFLAGS		= -mz80 --stack-auto --int-long-reent --float-reent

LD		= /usr/local/bin/link-z80
LDFLAGS		= 

ASMEXT		= .asm
OBJEXT		= .rel
LIBEXT		= .lib
EXEEXT		= .hex

ASRCS		= example.asm
AOBJS		= $(ASRCS:$(ASMEXT)=$(OBJEXT))

CSRCS		=
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(SSRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= example$(EXEEXT)

all: $(BIN)
default: $(BIN)

$(AOBJS): $(ASRCS)
	$(AS) $(ASFLAGS) $@ $<

$(COBJS) $(TESTOBJS): %$(OBJEXT): %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) $< -o $@
	
clean:
	@rm -f $(BIN) *.o *.rel *.lst *.sym *.adb *.ihx *.map *.mem *.rst *.lnk *~

distclean: clean
