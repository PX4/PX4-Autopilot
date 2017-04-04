/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <nuttx/arch.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <stm32_bbsram.h>

#include <systemlib/px4_macros.h>
#include <systemlib/hardfault_log.h>
#include <lib/version/version.h>


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
__EXPORT int hardfault_log_main(int argc, char *argv[]);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define OUT_BUFFER_LEN  200
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int genfault(int fault);
/****************************************************************************
 * Private Data
 ****************************************************************************/
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * genfault
 ****************************************************************************/

static int genfault(int fault)
{

	/* Pointer to System Control Block's  System Control Register */

	uint32_t *pCCR = (uint32_t *)0xE000ED14;

	static volatile int k = 0;

	switch (fault) {
	case 0:

		/* Enable divide by 0 fault generation */

		*pCCR |= 0x10;

		k =  1 / fault;

		/* This is not going to happen
		 * Enable divide by 0 fault generation
		 */

		*pCCR &= ~0x10;
		break;

	case 1:
		ASSERT(fault == 0);
		/* This is not going to happen */
		break;

	default:
		break;

	}

	UNUSED(k);
	return OK;
}


/****************************************************************************
 * format_fault_time
 ****************************************************************************/
/* Ensure Size is the same foe formats or rewrite this */
CCASSERT(TIME_FMT_LEN == HEADER_TIME_FMT_LEN);
static int format_fault_time(char *format, struct timespec *ts, char *buffer, unsigned int maxsz)
{
	int ret = -EINVAL;

	if (buffer != NULL && format != NULL) {
		ret = -ENOMEM;

		if (maxsz >= TIME_FMT_LEN + 1) {
			struct tm tt;
			time_t time_sec = ts->tv_sec + (ts->tv_nsec / 1e9);
			gmtime_r(&time_sec, &tt);

			if (TIME_FMT_LEN == strftime(buffer, maxsz, format, &tt)) {
				ret = OK;
			}
		}
	}

	return ret;
}

/****************************************************************************
 * format_fault_file_name
 ****************************************************************************/

static int format_fault_file_name(struct timespec *ts, char *buffer, unsigned int maxsz)
{
	char fmtbuff[ TIME_FMT_LEN + 1];
	int ret = -EINVAL;

	if (buffer) {

		ret = -ENOMEM;
		unsigned int plen = LOG_PATH_BASE_LEN;

		if (maxsz >= LOG_PATH_LEN) {
			strncpy(buffer, LOG_PATH_BASE, plen);
			maxsz -= plen;
			int rv = format_fault_time(TIME_FMT, ts, fmtbuff, arraySize(fmtbuff));

			if (rv == OK) {
				int n = snprintf(&buffer[plen], maxsz, LOG_NAME_FMT, fmtbuff);

				if (n == (int) LOG_NAME_LEN + TIME_FMT_LEN) {
					ret = OK;
				}
			}
		}
	}

	return ret;
}

/****************************************************************************
 * identify
 ****************************************************************************/
static void identify(char *caller)
{
	if (caller) {
		syslog(LOG_INFO, "[%s] ", caller);
	}
}


/****************************************************************************
 * hardfault_get_desc
 ****************************************************************************/
static int hardfault_get_desc(char *caller, struct bbsramd_s *desc, bool silent)
{
	int ret = -ENOENT;
	int fd = open(HARDFAULT_PATH, O_RDONLY);

	if (fd < 0) {
		if (!silent) {
			identify(caller);
			syslog(LOG_INFO, "Failed to open Fault Log file [%s] (%d)\n", HARDFAULT_PATH, fd);
		}

	} else {
		ret = -EIO;
		int rv = ioctl(fd, PX4_BBSRAM_GETDESC_IOCTL, (unsigned long)((uintptr_t)desc));

		if (rv >= 0) {
			ret = fd;

		} else {
			identify(caller);
			syslog(LOG_INFO, "Failed to get Fault Log descriptor (%d)\n", rv);
		}
	}

	return ret;
}

/****************************************************************************
 * write_stack_detail
 ****************************************************************************/
static int write_stack_detail(bool inValid, _stack_s *si, char *sp_name,
			      char *buffer, int max, int fd)
{

	int n = 0;
	uint32_t sbot = si->top - si->size;
	n =   snprintf(&buffer[n], max - n, " %s stack: \n", sp_name);
	n +=  snprintf(&buffer[n], max - n, "  top:    0x%08x\n", si->top);
	n +=  snprintf(&buffer[n], max - n, "  sp:     0x%08x %s\n", si->sp, (inValid ? "Invalid" : "Valid"));

	if (n != write(fd, buffer, n)) {
		return -EIO;
	}

	n = 0;
	n +=  snprintf(&buffer[n], max - n, "  bottom: 0x%08x\n", sbot);
	n +=  snprintf(&buffer[n], max - n, "  size:   0x%08x\n",  si->size);

	if (n != write(fd, buffer, n)) {
		return -EIO;
	}

#ifdef CONFIG_STACK_COLORATION
	FAR struct tcb_s tcb;
	tcb.stack_alloc_ptr = (void *) sbot;
	tcb.adj_stack_size = si->size;
	n = snprintf(buffer, max,         "  used:   %08x\n", up_check_tcbstack(&tcb));

	if (n != write(fd, buffer, n)) {
		return -EIO;
	}

#endif
	return OK;
}

/****************************************************************************
 * write_stack
 ****************************************************************************/
static int read_stack(int fd, stack_word_t *words, int num)
{
	int bytes = read(fd, (char *) words, sizeof(stack_word_t) * num);

	if (bytes > 0) {
		bytes /= sizeof(stack_word_t);
	}

	return bytes;
}
static int  write_stack(bool inValid, int winsize, uint32_t wtopaddr,
			uint32_t topaddr, uint32_t spaddr, uint32_t botaddr,
			char *sp_name, char *buffer, int max, int infd, int outfd)
{
	char marker[30];
	stack_word_t stack[32];
	int ret = OK;

	int n = snprintf(buffer, max, "%s memory region, stack pointer lies %s stack\n",
			 sp_name, (inValid ? "outside of" : "within"));

	if (n != write(outfd, buffer, n)) {

		ret = -EIO;

	} else {

		while (winsize > 0 && ret == OK) {
			int chunk = read_stack(infd, stack, arraySize(stack));

			if (chunk <= 0) {
				ret = -EIO;

			} else {
				winsize -= chunk;

				for (int i = 0; i < chunk; i++) {
					if (wtopaddr == topaddr) {
						strncpy(marker, "<-- ", sizeof(marker));
						strncat(marker, sp_name, sizeof(marker));
						strncat(marker, " top", sizeof(marker));

					} else if (wtopaddr == spaddr) {
						strncpy(marker, "<-- ", sizeof(marker));
						strncat(marker, sp_name, sizeof(marker));

					} else if (wtopaddr == botaddr) {
						strncpy(marker, "<-- ", sizeof(marker));
						strncat(marker, sp_name, sizeof(marker));
						strncat(marker, " bottom", sizeof(marker));

					} else {
						marker[0] = '\0';
					}

					n = snprintf(buffer, max, "0x%08x 0x%08x%s\n", wtopaddr, stack[i], marker);

					if (n != write(outfd, buffer, n)) {
						ret = -EIO;
					}

					wtopaddr--;
				}
			}
		}
	}

	return ret;
}

/****************************************************************************
 * write_registers
 ****************************************************************************/
static int write_registers(uint32_t regs[], char *buffer, int max, int fd)
{
	int n = snprintf(buffer, max, " r0:0x%08x r1:0x%08x  r2:0x%08x  r3:0x%08x  r4:0x%08x  r5:0x%08x r6:0x%08x r7:0x%08x\n",
			 regs[REG_R0],  regs[REG_R1],
			 regs[REG_R2],  regs[REG_R3],
			 regs[REG_R4],  regs[REG_R5],
			 regs[REG_R6],  regs[REG_R7]);

	if (n != write(fd, buffer, n)) {
		return -EIO;
	}

	n  = snprintf(buffer, max, " r8:0x%08x r9:0x%08x r10:0x%08x r11:0x%08x r12:0x%08x  sp:0x%08x lr:0x%08x pc:0x%08x\n",
		      regs[REG_R8],  regs[REG_R9],
		      regs[REG_R10], regs[REG_R11],
		      regs[REG_R12], regs[REG_R13],
		      regs[REG_R14], regs[REG_R15]);

	if (n != write(fd, buffer, n)) {
		return -EIO;
	}

#ifdef CONFIG_ARMV7M_USEBASEPRI
	n = snprintf(buffer, max, " xpsr:0x%08x basepri:0x%08x control:0x%08x\n",
		     regs[REG_XPSR],  regs[REG_BASEPRI],
		     getcontrol());
#else
	n = snprintf(buffer, max, " xpsr:0x%08x primask:0x%08x control:0x%08x\n",
		     regs[REG_XPSR],  regs[REG_PRIMASK],
		     getcontrol());
#endif

	if (n != write(fd, buffer, n)) {
		return -EIO;
	}

#ifdef REG_EXC_RETURN
	n = snprintf(buffer, max, " exe return:0x%08x\n", regs[REG_EXC_RETURN]);

	if (n != write(fd, buffer, n)) {
		return -EIO;
	}

#endif
	return OK;
}

/****************************************************************************
 * write_registers_info
 ****************************************************************************/
static int write_registers_info(int fdout, info_s *pi, char *buffer, int sz)
{
	int ret = ENOENT;

	if (pi->flags & eRegsPresent) {
		ret = -EIO;
		int n = snprintf(buffer, sz, " Processor registers: from 0x%08x\n", pi->current_regs);

		if (n == write(fdout, buffer, n)) {
			ret = write_registers(pi->regs, buffer, sz, fdout);
		}
	}

	return ret;
}

/****************************************************************************
 * write_interrupt_stack_info
 ****************************************************************************/
static int write_interrupt_stack_info(int fdout, info_s *pi, char *buffer,
				      unsigned int sz)
{
	int ret = ENOENT;

	if (pi->flags & eIntStackPresent) {
		ret = write_stack_detail((pi->flags & eInvalidIntStackPrt) != 0,
					 &pi->stacks.interrupt, "IRQ",
					 buffer, sz, fdout);
	}

	return ret;
}

/****************************************************************************
 * write_user_stack_info
 ****************************************************************************/
static int write_user_stack_info(int fdout, info_s *pi, char *buffer,
				 unsigned int sz)
{
	int ret = ENOENT;

	if (pi->flags & eUserStackPresent) {
		ret = write_stack_detail((pi->flags & eInvalidUserStackPtr) != 0,
					 &pi->stacks.user, "User", buffer, sz, fdout);
	}

	return ret;
}

/****************************************************************************
 * write_dump_info
 ****************************************************************************/
static int write_dump_info(int fdout, info_s *info, struct bbsramd_s *desc,
			   char *buffer, unsigned int sz)
{
	char fmtbuff[ TIME_FMT_LEN + 1];
	format_fault_time(HEADER_TIME_FMT, &desc->lastwrite, fmtbuff, sizeof(fmtbuff));

	bool isFault = (info->current_regs != 0 || info->pid == 0);
	int n;
	n = snprintf(buffer, sz, "System fault Occurred on: %s\n", fmtbuff);

	if (n != write(fdout, buffer, n)) {
		return -EIO;
	}

	if (isFault) {
		n = snprintf(buffer, sz, " Type:Hard Fault");

	} else {
		n = snprintf(buffer, sz, " Type:Assertion failed");
	}

	if (n != write(fdout, buffer, n)) {
		return -EIO;
	}

#ifdef CONFIG_TASK_NAME_SIZE
	n = snprintf(buffer, sz, " in file:%s at line: %d running task: %s\n",
		     info->filename, info->lineno, info->name);
#else
	n = snprintf(buffer, sz, " in file:%s at line: %d \n",
		     info->filename, info->lineno);
#endif

	if (n != write(fdout, buffer, n)) {
		return -EIO;
	}

	n = snprintf(buffer, sz, " FW git-hash: %s\n", px4_firmware_version_string());

	if (n != write(fdout, buffer, n)) {
		return -EIO;
	}

	n = snprintf(buffer, sz, " Build datetime: %s %s\n", __DATE__, __TIME__);

	if (n != write(fdout, buffer, n)) {
		return -EIO;
	}

	n = snprintf(buffer, sz, " Build url: %s \n", px4_build_uri());

	if (n != write(fdout, buffer, n)) {
		return -EIO;
	}

	return OK;
}

/****************************************************************************
 * write_dump_time
 ****************************************************************************/
static int write_dump_time(char *caller, char *tag, int fdout,
			   struct timespec *ts, char *buffer, unsigned int sz)
{
	int ret = OK;
	char fmtbuff[ TIME_FMT_LEN + 1];
	format_fault_time(HEADER_TIME_FMT, ts, fmtbuff, sizeof(fmtbuff));
	int n = snprintf(buffer, sz, "[%s] -- %s %s Fault Log --\n", caller, fmtbuff, tag);

	if (n != write(fdout, buffer, n)) {
		ret = -EIO;
	}

	return ret;
}
/****************************************************************************
 * write_dump_footer
 ****************************************************************************/
static int write_dump_header(char *caller, int fdout, struct timespec *ts,
			     char *buffer, unsigned int sz)
{
	return write_dump_time(caller, "Begin", fdout, ts, buffer, sz);
}
/****************************************************************************
 * write_dump_footer
 ****************************************************************************/
static int write_dump_footer(char *caller, int fdout, struct timespec *ts,
			     char *buffer, unsigned int sz)
{
	return write_dump_time(caller, "END", fdout, ts, buffer, sz);
}
/****************************************************************************
 * write_intterupt_satck
 ****************************************************************************/
static int write_intterupt_stack(int fdin, int fdout, info_s *pi, char *buffer,
				 unsigned int sz)
{
	int ret = ENOENT;

	if ((pi->flags & eIntStackPresent) != 0) {
		lseek(fdin, offsetof(fullcontext_s, istack), SEEK_SET);
		ret = write_stack((pi->flags & eInvalidIntStackPrt) != 0,
				  CONFIG_ISTACK_SIZE,
				  pi->stacks.interrupt.sp + CONFIG_ISTACK_SIZE / 2,
				  pi->stacks.interrupt.top,
				  pi->stacks.interrupt.sp,
				  pi->stacks.interrupt.top - pi->stacks.interrupt.size,
				  "Interrupt sp", buffer, sz, fdin, fdout);
	}

	return ret;
}


/****************************************************************************
 * write_user_stack
 ****************************************************************************/
static int write_user_stack(int fdin, int fdout, info_s *pi, char *buffer,
			    unsigned int sz)
{
	int ret = ENOENT;

	if ((pi->flags & eUserStackPresent) != 0) {
		lseek(fdin, offsetof(fullcontext_s, ustack), SEEK_SET);
		ret = write_stack((pi->flags & eInvalidUserStackPtr) != 0,
				  CONFIG_USTACK_SIZE,
				  pi->stacks.user.sp + CONFIG_USTACK_SIZE / 2,
				  pi->stacks.user.top,
				  pi->stacks.user.sp,
				  pi->stacks.user.top - pi->stacks.user.size,
				  "User sp", buffer, sz, fdin, fdout);
	}

	return ret;
}

/****************************************************************************
 * commit
 ****************************************************************************/
static int hardfault_commit(char *caller)
{
	int ret = -ENOENT;
	int state = -1;
	struct bbsramd_s desc;
	char path[LOG_PATH_LEN + 1];
	ret = hardfault_get_desc(caller, &desc, false);

	if (ret >= 0) {

		int fd = ret;
		state = (desc.lastwrite.tv_sec || desc.lastwrite.tv_nsec) ?  OK : 1;
		int rv = close(fd);

		if (rv < 0) {
			identify(caller);
			syslog(LOG_INFO, "Failed to Close Fault Log (%d)\n", rv);

		} else {

			if (state != OK) {
				identify(caller);
				syslog(LOG_INFO, "Nothing to save\n", path);
				ret = -ENOENT;

			} else {
				ret = format_fault_file_name(&desc.lastwrite, path, arraySize(path));

				if (ret == OK) {
					int fdout = open(path, O_RDWR | O_CREAT);

					if (fdout > 0) {
						identify(caller);
						syslog(LOG_INFO, "Saving Fault Log file %s\n", path);
						ret = hardfault_write(caller, fdout, HARDFAULT_FILE_FORMAT, true);
						identify(caller);
						syslog(LOG_INFO, "Done saving Fault Log file\n");
						close(fdout);
					}
				}
			}
		}
	}

	return ret;
}


/****************************************************************************
 * hardfault_dowrite
 ****************************************************************************/
static int hardfault_dowrite(char *caller, int infd, int outfd,
			     struct bbsramd_s *desc, int format)
{
	int ret = -ENOMEM;
	char *line = zalloc(OUT_BUFFER_LEN);

	if (line) {
		char *info = zalloc(sizeof(info_s));

		if (info) {
			lseek(infd, offsetof(fullcontext_s, info), SEEK_SET);
			ret = read(infd, info, sizeof(info_s));

			if (ret < 0) {
				identify(caller);
				syslog(LOG_INFO, "Failed to read Fault Log file [%s] (%d)\n", HARDFAULT_PATH, ret);
				ret = -EIO;

			} else {
				info_s *pinfo = (info_s *) info;
				ret = write_dump_header(caller, outfd, &desc->lastwrite, line, OUT_BUFFER_LEN);

				if (ret == OK) {

					switch (format) {
					case HARDFAULT_DISPLAY_FORMAT:
						ret = write_intterupt_stack(infd, outfd, pinfo, line, OUT_BUFFER_LEN);

						if (ret >= OK) {
							ret = write_user_stack(infd, outfd, pinfo, line, OUT_BUFFER_LEN);

							if (ret >= OK) {
								ret = write_dump_info(outfd, pinfo, desc, line, OUT_BUFFER_LEN);

								if (ret >= OK) {
									ret = write_registers_info(outfd, pinfo, line, OUT_BUFFER_LEN);

									if (ret >= OK) {
										ret = write_interrupt_stack_info(outfd, pinfo, line, OUT_BUFFER_LEN);

										if (ret >= OK) {
											ret = write_user_stack_info(outfd, pinfo, line, OUT_BUFFER_LEN);
										}
									}
								}
							}
						}

						break;

					case HARDFAULT_FILE_FORMAT:
						ret = write_dump_info(outfd, pinfo, desc, line, OUT_BUFFER_LEN);

						if (ret == OK) {
							ret = write_registers_info(outfd, pinfo, line, OUT_BUFFER_LEN);

							if (ret >= OK) {
								ret = write_interrupt_stack_info(outfd, pinfo, line, OUT_BUFFER_LEN);

								if (ret >= OK) {
									ret = write_user_stack_info(outfd, pinfo, line, OUT_BUFFER_LEN);

									if (ret >= OK) {
										ret = write_intterupt_stack(infd, outfd, pinfo, line, OUT_BUFFER_LEN);

										if (ret >= OK) {
											ret = write_user_stack(infd, outfd, pinfo, line, OUT_BUFFER_LEN);
										}
									}
								}
							}
						}

						break;

					default:
						ret = -EINVAL;
						break;
					}
				}

				if (ret >= OK) {
					ret = write_dump_footer(caller, outfd, &desc->lastwrite, line, OUT_BUFFER_LEN);
				}
			}

			free(info);
		}

		free(line);
	}

	return ret;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * hardfault_rearm
 ****************************************************************************/
__EXPORT int hardfault_rearm(char *caller)
{
	int ret = OK;
	int rv = unlink(HARDFAULT_PATH);

	if (rv < 0) {
		identify(caller);
		syslog(LOG_INFO, "Failed to re arming Fault Log (%d)\n", rv);
		ret = -EIO;

	} else {
		identify(caller);
		syslog(LOG_INFO, "Fault Log is Armed\n");
	}

	return ret;
}

/****************************************************************************
 * hardfault_check_status
 ****************************************************************************/
__EXPORT int hardfault_check_status(char *caller)
{
	int state = -1;
	struct bbsramd_s desc;
	int ret = hardfault_get_desc(caller, &desc, true);

	if (ret < 0) {
		identify(caller);

		if (ret == -ENOENT) {
			syslog(LOG_INFO, "Fault Log is Armed\n");

		} else {
			syslog(LOG_INFO, "Failed to open Fault Log file [%s] (%d)\n", HARDFAULT_PATH, ret);
		}

	} else {
		int fd = ret;
		state = (desc.lastwrite.tv_sec || desc.lastwrite.tv_nsec) ?  OK : 1;
		int rv = close(fd);

		if (rv < 0) {
			identify(caller);
			syslog(LOG_INFO, "Failed to Close Fault Log (%d)\n", rv);

		} else {
			ret = state;
			identify(caller);
			syslog(LOG_INFO, "Fault Log info File No %d Length %d flags:0x%02x state:%d\n",
			       (unsigned int)desc.fileno, (unsigned int) desc.len, (unsigned int)desc.flags, state);

			if (state == OK) {
				char buf[TIME_FMT_LEN + 1];
				format_fault_time(HEADER_TIME_FMT, &desc.lastwrite, buf, arraySize(buf));
				identify(caller);
				syslog(LOG_INFO, "Fault Logged on %s - Valid\n", buf);

			} else {
				rv = hardfault_rearm(caller);

				if (rv < 0) {
					ret = rv;
				}
			}
		}
	}

	return ret;
}

/****************************************************************************
 * hardfault_increment_reboot
 ****************************************************************************/
__EXPORT int hardfault_increment_reboot(char *caller, bool reset)
{
	int ret = -EIO;
	int count = 0;
	int fd = open(HARDFAULT_REBOOT_PATH, O_RDWR | O_CREAT);

	if (fd < 0) {
		identify(caller);
		syslog(LOG_INFO, "Failed to open Fault reboot count file [%s] (%d)\n", HARDFAULT_REBOOT_PATH, ret);

	} else {

		ret = OK;

		if (!reset) {
			if (read(fd, &count, sizeof(count)) !=  sizeof(count)) {
				ret = -EIO;
				close(fd);

			} else {
				lseek(fd, 0, SEEK_SET);
				count++;
			}
		}

		if (ret == OK) {
			ret = write(fd, &count, sizeof(count));

			if (ret != sizeof(count)) {
				ret = -EIO;

			} else {
				ret = close(fd);

				if (ret == OK) {
					ret = count;
				}
			}
		}
	}

	return ret;
}
/****************************************************************************
 * hardfault_write
 ****************************************************************************/

__EXPORT int hardfault_write(char *caller, int fd, int format, bool rearm)
{
	struct bbsramd_s desc;

	switch (format) {

	case HARDFAULT_FILE_FORMAT:
	case HARDFAULT_DISPLAY_FORMAT:
		break;

	default:
		return -EINVAL;
	}

	int ret = hardfault_get_desc(caller, &desc, false);

	if (ret >= 0) {
		int hffd = ret;


		int rv =  hardfault_dowrite(caller, hffd, fd, &desc, format);

		ret = close(hffd);

		if (ret < 0) {
			identify(caller);
			syslog(LOG_INFO, "Failed to Close Fault Log (%d)\n", ret);

		}

		if (rv == OK && rearm) {
			ret = hardfault_rearm(caller);

			if (ret < 0) {
				identify(caller);
				syslog(LOG_INFO, "Failed to re-arm Fault Log (%d)\n", ret);
			}
		}

		if (ret == OK) {
			ret = rv;
		}

		if (ret != OK) {
			identify(caller);
			syslog(LOG_INFO, "Failed to Write Fault Log (%d)\n", ret);
		}
	}

	return ret;
}

/****************************************************************************
 * Name: hardfault_log_main
 ****************************************************************************/
__EXPORT int hardfault_log_main(int argc, char *argv[])
{
	char *self = "hardfault_log";

	if (!strcmp(argv[1], "check")) {

		return hardfault_check_status(self);

	} else if (!strcmp(argv[1], "rearm")) {

		return hardfault_rearm(self);

	} else if (!strcmp(argv[1], "fault")) {

		int fault = 0;

		if (argc > 2) {
			fault = atol(argv[2]);
		}

		return genfault(fault);

	} else  if (!strcmp(argv[1], "commit")) {

		return hardfault_commit(self);

	} else  if (!strcmp(argv[1], "count")) {

		return hardfault_increment_reboot(self, false);

	} else  if (!strcmp(argv[1], "reset")) {

		return hardfault_increment_reboot(self, true);

	}

	fprintf(stderr, "unrecognised command, try 'check' ,'rearm' , 'fault', 'count', 'reset' or 'commit'\n");
	return -EINVAL;
}
