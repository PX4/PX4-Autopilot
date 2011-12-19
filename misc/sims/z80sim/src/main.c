#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <errno.h>

#include "Z80.h"

/* This is the simulated z80 16-bit address space */

static unsigned char memory[65536];

/* This is the simulatin execution context */

static Z80 gR;

/* Command line options */

static int gtrace = 0;
static const char *gfilename = NULL;

/* Name:        RdZ80 and WrZ80
 * Description: These functions are called when access to RAM occurs.
 *              They allow to control memory access.
 */

void WrZ80(register word Addr, register byte Value)
{
	memory[Addr] = Value;
}

byte RdZ80(register word Addr)
{
	return memory[Addr];
}

/* Name:        InZ80 and OutZ80
 * Description: Z80 emulation calls these functions to read/write from
 *              I/O ports. There can be 65536 I/O ports, but only first
 *              256 are usually used.
 */

void OutZ80(register word Port,register byte Value)
{
	/* We recognize only one port, 0xbe, which is mapped to stdout */

	if ((Port & 0x00ff) == 0xbe)
	{
		putchar(Value);
		fflush(stdout);
	}
}

byte InZ80(register word Port)
{
	/* We recognize only one port, 0xbe, which is mapped to stdin */

	if ((Port & 0x00ff) == 0xbe)
	{
		return getchar();
	}
	return 0;
}

/* Name:        PatchZ80
 * Description: Z80 emulation calls this function when it encounters a
 *              special patch command (ED FE) provided for user needs.
 *              For example, it can be called to emulate BIOS calls,
 *              such as disk and tape access. Replace it with an empty
 *              macro for no patching.
 */

void PatchZ80(register Z80 *R)
{
}

/* Name:        LoopZ80
 * Description: Z80 emulation calls this function periodically to check
 *              if the system hardware requires any interrupts. This
 *              function must return an address of the interrupt vector
 *              (0x0038, 0x0066, etc.) or INT_NONE for no interrupt.
 *               Return INT_QUIT to exit the emulation loop.
 */

word LoopZ80(register Z80 *R)
{
	return INT_NONE;
}

/* Name:        JumpZ80
 * Description: Z80 emulation calls this function when it executes a
 *              JP, JR, CALL, RST, or RET. You can use JumpZ80() to
 *              trap these opcodes and switch memory layout.
 */

#ifdef JUMPZ80
void JumpZ80(word PC)
{
	if (gtrace)
	{
		printf("PC: %04X [%02X] HL: %04X SP: %04X [%04X, %04X, ...]\n",
			gR.PC.W, memory[gR.PC.W], gR.HL.W, gR.SP.W,
			((int)memory[gR.SP.W]   | ((int)memory[gR.SP.W+1]) << 8),
			((int)memory[gR.SP.W+2] | ((int)memory[gR.SP.W+3]) << 8));
	}
}
#endif

/* Intel hex code based largely on code taken from the PJRC website.
 * Licensing requires the following:
 *
 * Author:  Paul Stoffregen
 * Contact: paul@ece.orst.edu
 */

/* Name:        parse_hex
 * Description: Parse one line from an Intel hex file
 */

static int parse_hex(const char *hex, unsigned char *binary, int *addr, int *nbytes, int *code)
{
	const char *ptr;
	int accum;
	int chksum;
	int len;
	
	*nbytes = 0;

	/* Each valid hex begins with a colon */

	ptr = hex;
	if (*hex != ':')
	{
		return 0;
	}
	ptr++;

	/* The minimun size is ':'(1) + (0) + addr(4) + type(2) + data(2) + chechksum(2) = 11 */

	if (strlen(hex) < 11)
	{
		return 0;
	}

	/* Get the length byte */

	if (!sscanf(ptr, "%02x", &len))
	{
		return 0;
	}
	ptr += 2;

	/* Now verify the length is ':'(1) + l2*len + addr(4) + type(2) + data(2) + chechksum(2) */

	if (strlen(hex) < (11 + (2*len)))
	{
		return 0;
	}

	/* Get the address */

	if (!sscanf(ptr, "%04x", addr))
	{
		return 0;
	}
	ptr += 4;

	/* Get the code byte */

	if (!sscanf(ptr, "%02x", code))
	{
		return 0;
	}
	ptr += 2;

	/* Copy the data and calculate the chechksum */

	accum = len + (*addr >> 8) + *addr + *code;
	while (*nbytes < len)
	{
		int tmp;

		/* Get the next data byte */

		if (!sscanf(ptr, "%02x", &tmp))
		{
			return 0;
		}
		ptr += 2;

		/* Transfer the data to the user binary */

		binary[*nbytes] = tmp;

		/* Update the accum */

		accum += binary[*nbytes];
		(*nbytes)++;
	}

	/* Get the checksum */

	if (!sscanf(ptr, "%02x", &chksum))
	{
		return 0;
	}

	/* Verify the checksum */

	if (((accum + chksum) & 0xff) != 0)
	{
		return 0;
	}
	return 1;
}

/* Name:        load_file
 * Description: Read an entire Intel hex file into (simulated) z80 memory
 */

int load_file(const char *filename)
{
	char hex[1000];
	FILE *stream;
	unsigned char binary[256];
	int i;
	int total = 0;
	int lineno = 1;
	int minaddr = 65536;
	int maxaddr = 0;
	int addr;
	int nbytes;
	int status;

	/* Open the ascii hex file */

	stream = fopen(filename, "r");
	if (stream == NULL)
	{
		printf("ERROR: Failed to open file '%s' for reading: %s\n", filename, strerror(errno));
		return 0;
	}

	/* Loop until every line has been read */

	while (!feof(stream) && !ferror(stream))
	{
		/* Read the next line from the Intel hex file */

		hex[0] = '\0';
		fgets(hex, 1000, stream);

		/* Remove any trailing CR/LF */

		if (hex[strlen(hex)-1] == '\n')
		{
			hex[strlen(hex)-1] = '\0';
		}

		if (hex[strlen(hex)-1] == '\r')
		{
			hex[strlen(hex)-1] = '\0';
		}

		/* Parse the hex line */

		if (parse_hex(hex, binary, &addr, &nbytes, &status))
		{
			/* Valid data? */

			if (status == 0)
			{
				/* Yes.. move it into the z80 memory image */

				for (i = 0; i <= (nbytes-1); i++)
				{
					memory[addr] = binary[i];
					total++;

					/* Keep track of the highest and lowest addresses written */

					if (addr < minaddr)
					{
						minaddr = addr;
					}

					if (addr > maxaddr)
					{
						maxaddr = addr;
					}
					addr++;
				}
			}

			/* End of file? */

			else if (status == 1)
			{
				fclose(stream);
				printf("Loaded %d bytes between %04x to %04x\n", total, minaddr, maxaddr);
				return 1;
			}
			else if (status != 2)  /* begin of file */
			{
				printf("ERROR: Unrecognized status=%d at line=%d\n", status, lineno);
				return 0;
			}
		}
		else
		{
			printf("ERROR: Failed to parse %s at line: %d\n", filename, lineno);
			return 0;
		}
		lineno++;
	}
	printf("ERROR: No end of file marker encountered in %s\n", filename);
	return 0;
}

/* Name:        sighandler
 * Description: Catch program termination via control-C
 */

void sighandler(int signo)
{
	sigset_t set;
	char command[80];
	int i;
	int j;

	sigemptyset(&set);
	sigaddset(&set, SIGINT);
	sigprocmask(SIG_UNBLOCK, &set, NULL);
	signal(SIGINT, SIG_DFL);

	printf("AF:%04X HL:%04X DE:%04X BC:%04X PC:%04X SP:%04X IX:%04X IY:%04X I:%02X\n",
		gR.AF.W, gR.HL.W, gR.DE.W, gR.BC.W, gR.PC.W, gR.SP.W, gR.IX.W, gR.IY.W, gR.I);

	printf("AT PC: [%02X]  AT SP: [%04X]  %s: %s\n",
		RdZ80(gR.PC.W), RdZ80(gR.SP.W) + RdZ80(gR.SP.W+1) * 256,
		gR.IFF & 0x04? "IM2" : gR.IFF & 0x02? "IM1" : "IM0",
		gR.IFF & 0x01? "EI" : "DI");

	for (;;)
	{
		printf("\n[Command,'?']-> ");
		fflush(stdout);
		fflush(stdin);

		fgets(command, 50, stdin);

		switch(command[0])
		{
		case 'H':
		case 'h':
		case '?':
			puts("\n***** Built-in Z80 Debugger Commands *****");
			puts("m <addr>   : Memory dump at addr");
			puts("?,h        : Show this help text");
			puts("q          : Exit Z80 emulation");
			break;

		case 'M':
		case 'm':
			{
				unsigned short addr;

				if (strlen(command) > 1)
				{
					sscanf(command+1, "%hX", &addr);
				}
				else
				{
					addr = gR.PC.W;
				}

				puts("");
				for (j = 0; j < 16; j++)
				{
					printf("%04X: ",addr);
					for (i = 0; i < 16; i++, addr++)
					{
						printf("%02X ", memory[addr]);
					}
					printf(" | ");
					addr -= 16;
					for (i = 0; i < 16; i++, addr++)
					{
						putchar(isprint(memory[addr])? memory[addr]:'.');
					}
					puts("");
				}
			}
			break;

		case 'Q':
		case 'q':
			exit(0);
		}
	}
	exit(0);
}

static void show_usage(const char *progname, int exitcode)
{
	fprintf(stderr, "\nUSAGE: %s [OPTIONS] <Intel-Hex-File>\n", progname);
	fprintf(stderr, "\nWhere [OPTIONS] include:\n");
	fprintf(stderr, "\n\t-t\tEnable trace output\n");
	fprintf(stderr, "\t-h\tShow this message\n");
	exit(exitcode);
}

static void parse_commandline(int argc, char **argv)
{
	int opt;

	while ((opt = getopt(argc, argv, ":th")) != -1)
	{
		switch (opt)
		{
		case 't':
			gtrace++;
			break;
		case 'h':
			show_usage(argv[0], 0);
			break;
		case '?':
			fprintf(stderr, "ERROR: Unrecognized option: %c\n", optopt);
			show_usage(argv[0], 1);
			break;
		case ':':
			fprintf(stderr, "ERROR: Missing argument to option: %c\n", optopt);
			show_usage(argv[0], 1);
			break;
		}
	}

	if (optind >= argc)
	{
		fprintf(stderr, "ERROR: Missing filename argument\n");
		show_usage(argv[0], 1);
	}

	gfilename = argv[optind];
	optind++;

	if (optind < argc)
	{
		fprintf(stderr, "ERROR: Extra stuff on command line after filename\n");
		show_usage(argv[0], 1);
	}
}

/* Name:        main
 * Description: Program entry point
 */

int main(int argc, char **argv, char **envp)
{
	/* Parse the command line options */

	parse_commandline(argc, argv);

	/* Set all simulated z80 memory to a known value and load the Intel hex file */

	memset(memory, 0, 65536);
	load_file(gfilename);

	/* Configure the simulation */

	memset(&gR, 0, sizeof(Z80));
	gR.IPeriod    = 10000; /* 100Hz at 10MHz */
	gR.IAutoReset = 1;

	/* Set up to catch SIGINT (control-C from console) */

	(void)signal(SIGINT, sighandler);

	/* Then start the simulation */

	RunZ80(&gR);
	return 0;
}
