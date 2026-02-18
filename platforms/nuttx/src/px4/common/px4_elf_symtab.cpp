/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_elf_symtab.cpp
 *
 * Export symbol table for loadable ELF modules on NuttX.
 *
 * This table exports PX4 and NuttX/libc APIs so that ELF binaries
 * loaded from the SD card can resolve their external references.
 *
 * IMPORTANT: Entries MUST be sorted alphabetically by symbol name
 * (CONFIG_SYMTAB_ORDEREDBYNAME=y enables binary search).
 */

#include <nuttx/config.h>

#ifdef CONFIG_ELF

#include <nuttx/symtab.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sched.h>
#include <semaphore.h>
#include <poll.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/printload.h>
#include <px4_platform/cpuload.h>
#include <uORB/uORB.h>
// Note: uORBTopics.hpp defines orb_topics_count() as static constexpr inline,
// so it gets compiled into each module directly — no export needed.
#include <uORB/uORBTopics.h>

// Declare PX4 module usage functions without including module.h
// (module.h defines macros that conflict with function pointer usage)
extern "C" {

extern const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1];
extern void px4_log_modulename(int level, const char *module_name, const char *fmt, ...);
extern void px4_log_raw(int level, const char *fmt, ...);

// Note: PRINT_MODULE_DESCRIPTION is an empty inline on NuttX (saves Flash) so no export needed.
void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category);
void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory);
void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category);
void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description);
void PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(bool i2c_support, bool spi_support);
void PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(uint8_t default_address);
void PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG(void);
void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
				  const char *description, bool is_optional);
void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
				    const char *description, bool is_optional);
void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional);
void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
				     const char *description, bool is_optional);
void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment);
void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional);

// ARM compiler runtime functions (from libgcc)
extern void __aeabi_uldivmod();
extern void __aeabi_ldivmod();

} // extern "C"

/**
 * Exported symbol table for loadable ELF modules.
 * Sorted alphabetically by name for binary search.
 *
 * Note: PRINT_MODULE_USAGE_COMMAND and PRINT_MODULE_USAGE_DEFAULT_COMMANDS
 * are macros (not functions) and are expanded at compile time in the module,
 * so they don't need to be exported here. They call PRINT_MODULE_USAGE_COMMAND_DESCR
 * which IS exported.
 */
extern "C" __attribute__((used)) const struct symtab_s g_px4_exports[] = {
	// Sorted by ASCII: uppercase (A-Z) < underscore (_) < lowercase (a-z)
	{"PRINT_MODULE_USAGE_ARG",           (FAR const void *)PRINT_MODULE_USAGE_ARG},
	{"PRINT_MODULE_USAGE_COMMAND_DESCR", (FAR const void *)PRINT_MODULE_USAGE_COMMAND_DESCR},
	{"PRINT_MODULE_USAGE_NAME",          (FAR const void *)PRINT_MODULE_USAGE_NAME},
	{"PRINT_MODULE_USAGE_NAME_SIMPLE",   (FAR const void *)PRINT_MODULE_USAGE_NAME_SIMPLE},
	{"PRINT_MODULE_USAGE_PARAM_COMMENT", (FAR const void *)PRINT_MODULE_USAGE_PARAM_COMMENT},
	{"PRINT_MODULE_USAGE_PARAM_FLAG",    (FAR const void *)PRINT_MODULE_USAGE_PARAM_FLAG},
	{"PRINT_MODULE_USAGE_PARAM_FLOAT",   (FAR const void *)PRINT_MODULE_USAGE_PARAM_FLOAT},
	{"PRINT_MODULE_USAGE_PARAM_INT",     (FAR const void *)PRINT_MODULE_USAGE_PARAM_INT},
	{"PRINT_MODULE_USAGE_PARAM_STRING",  (FAR const void *)PRINT_MODULE_USAGE_PARAM_STRING},
	{"PRINT_MODULE_USAGE_SUBCATEGORY",   (FAR const void *)PRINT_MODULE_USAGE_SUBCATEGORY},
	{"__aeabi_ldivmod",                  (FAR const void *)__aeabi_ldivmod},
	{"__aeabi_uldivmod",                 (FAR const void *)__aeabi_uldivmod},
	{"__px4_log_level_str",              (FAR const void *)__px4_log_level_str},
	{"atoi",                             (FAR const void *)atoi},
	{"clock_gettime",                    (FAR const void *)clock_gettime},
	{"clock_settime",                    (FAR const void *)clock_settime},
	{"close",                            (FAR const void *)close},
	{"cpuload_monitor_stop",             (FAR const void *)cpuload_monitor_stop},
	{"dprintf",                          (FAR const void *)dprintf},
	{"exit",                             (FAR const void *)exit},
	{"fclose",                           (FAR const void *)fclose},
	{"fflush",                           (FAR const void *)fflush},
	{"fopen",                            (FAR const void *)fopen},
	{"fprintf",                          (FAR const void *)fprintf},
	{"fputs",                            (FAR const void *)fputs},
	{"free",                             (FAR const void *)free},
	{"fwrite",                           (FAR const void *)fwrite},
	{"hrt_absolute_time",                (FAR const void *)hrt_absolute_time},
	{"init_print_load",                  (FAR const void *)init_print_load},
	{"ioctl",                            (FAR const void *)ioctl},
	{"localtime_r",                      (FAR const void *)localtime_r},
	{"malloc",                           (FAR const void *)malloc},
	{"memcmp",                           (FAR const void *)memcmp},
	{"memcpy",                           (FAR const void *)memcpy},
	{"memmove",                          (FAR const void *)memmove},
	{"memset",                           (FAR const void *)memset},
	{"nanosleep",                        (FAR const void *)nanosleep},
	{"open",                             (FAR const void *)open},
	{"orb_copy",                         (FAR const void *)orb_copy},
	{"orb_exists",                       (FAR const void *)orb_exists},
	{"orb_get_topics",                   (FAR const void *)orb_get_topics},
	{"orb_print_message_internal",       (FAR const void *)orb_print_message_internal},
	{"orb_set_interval",                 (FAR const void *)orb_set_interval},
	{"orb_subscribe",                    (FAR const void *)orb_subscribe},
	{"orb_subscribe_multi",              (FAR const void *)orb_subscribe_multi},
	{"orb_unsubscribe",                  (FAR const void *)orb_unsubscribe},
	{"poll",                             (FAR const void *)poll},
	{"print_load",                       (FAR const void *)print_load},
	{"printf",                           (FAR const void *)printf},
	{"puts",                             (FAR const void *)puts},
	{"px4_getopt",                       (FAR const void *)px4_getopt},
	{"px4_log_modulename",               (FAR const void *)px4_log_modulename},
	{"px4_log_raw",                      (FAR const void *)px4_log_raw},
	{"read",                             (FAR const void *)read},
	{"realloc",                          (FAR const void *)realloc},
	{"sched_lock",                       (FAR const void *)sched_lock},
	{"sched_unlock",                     (FAR const void *)sched_unlock},
	{"sem_destroy",                      (FAR const void *)sem_destroy},
	{"sem_init",                         (FAR const void *)sem_init},
	{"sem_post",                         (FAR const void *)sem_post},
	{"sem_wait",                         (FAR const void *)sem_wait},
	{"sleep",                            (FAR const void *)sleep},
	{"snprintf",                         (FAR const void *)snprintf},
	{"sprintf",                          (FAR const void *)sprintf},
	{"strcmp",                            (FAR const void *)strcmp},
	{"strftime",                         (FAR const void *)strftime},
	{"strlen",                           (FAR const void *)strlen},
	{"strncmp",                          (FAR const void *)strncmp},
	{"strncpy",                          (FAR const void *)strncpy},
	{"strstr",                           (FAR const void *)strstr},
	{"strtol",                           (FAR const void *)strtol},
	{"strtoul",                          (FAR const void *)strtoul},
	{"usleep",                           (FAR const void *)usleep},
	{"vsnprintf",                        (FAR const void *)vsnprintf},
	{"write",                            (FAR const void *)write},
};

extern "C" __attribute__((used)) const int g_px4_nexports = sizeof(g_px4_exports) / sizeof(struct symtab_s);

#endif /* CONFIG_ELF */
