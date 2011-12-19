arch/z80/src/z80
^^^^^^^^^^^^^^^^

The arch/z80 directories contain files to support a variety of 8-bit architectures
from ZiLOG (and spin-architectures such as the Rabbit2000).  The arch/z80/src/z80
sub-directory contains logic unique to the classic Z80 chip.

Files in this directory include:

z80_head.asm
	This is the main entry point into the Z80 program.  This includes the
	handler for the RESET, power-up interrupt vector and address zero and all
	RST interrupts.

z80_rom.asm
	Some architectures may have ROM located at address zero.  In this case, a
	special version of the "head" logic must be used.  This special "head"
	file is probably board-specific and, hence, belongs in the board-specific
	configs/<board-name>/src directory.  This file may, however, be used as
	a model for such a board-specific file.

	z80_rom.S is enabled by specifying CONFIG_LINKER_ROM_AT_0000 in the
	configuration file.

	A board specific version in the configs/<board-name>/src directory can be
	used by:
	
	1. Define CONFIG_ARCH_HAVEHEAD
	2. Add the board-specific head file, say <filename>.asm, to
	   configs/<board-name>/src
	3. Add a file called Make.defs in the configs/<board-name>/src directory
	   containing the line:  HEAD_ASRC = <file-name>.asm

Make.defs
	This is the standard makefile fragment that must be provided in all
	chip directories.  This fragment identifies the chip-specific file to
	be used in building libarch.

chip.h
	This is the standard header file that must be provided in all chip
	directories.

z80_initialstate.c, z80_copystate.c,  z80_restoreusercontext.asm, and
z80_saveusercontext.asm, switch
 	These files implement the Z80 context switching logic

z80_schedulesigaction.c and  z80_sigdeliver.c
	These files implement Z80 signal handling.
 