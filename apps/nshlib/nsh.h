/****************************************************************************
 * apps/nshlib/nsh.h
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#ifndef __APPS_NSHLIB_NSH_H
#define __APPS_NSHLIB_NSH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The background commands require pthread support */

#ifdef CONFIG_DISABLE_PTHREAD
#  ifndef CONFIG_NSH_DISABLEBG
#    define CONFIG_NSH_DISABLEBG 1
#  endif
#endif

/* Telnetd requires networking support */

#ifndef CONFIG_NET
#  undef CONFIG_NSH_TELNET
#endif

/* One front end must be defined */

#if !defined(CONFIG_NSH_CONSOLE) && !defined(CONFIG_NSH_TELNET)
#  error "No NSH front end defined"
#endif

/* If a USB device is selected for the NSH console then we need to handle some
 * special start-up conditions.
 */

#undef HAVE_USB_CONSOLE
#if defined(CONFIG_USBDEV)

/* Check for a PL2303 serial console.  Use console device "/dev/console". */

#  if defined(CONFIG_PL2303) && defined(CONFIG_PL2303_CONSOLE)
#    define HAVE_USB_CONSOLE 1

/* Check for a CDC/ACM serial console.  Use console device "/dev/console". */

#  elif defined(CONFIG_CDCACM) && defined(CONFIG_CDCACM_CONSOLE)
#    define HAVE_USB_CONSOLE 1

/* Check for other USB console.  USB console device must be provided in CONFIG_NSH_CONDEV */

#  elif defined(CONFIG_NSH_USBCONSOLE)
#    define HAVE_USB_CONSOLE 1
#  endif
#endif

/* Defaults for the USB console */

#ifdef HAVE_USB_CONSOLE

/* The default USB console device minor number is 0*/

#  ifndef CONFIG_NSH_UBSDEV_MINOR
#    define CONFIG_NSH_UBSDEV_MINOR 0
#  endif

/* The default console device is always /dev/console */

#  ifndef CONFIG_NSH_USBCONDEV
#    define CONFIG_NSH_USBCONDEV "/dev/console"
#  endif

/* USB trace settings */

#ifdef CONFIG_NSH_USBDEV_TRACEINIT
#  define TRACE_INIT_BITS       (TRACE_INIT_BIT)
#else
#  define TRACE_INIT_BITS       (0)
#endif

#define TRACE_ERROR_BITS        (TRACE_DEVERROR_BIT|TRACE_CLSERROR_BIT)

#ifdef CONFIG_NSH_USBDEV_TRACECLASS
#  define TRACE_CLASS_BITS      (TRACE_CLASS_BIT|TRACE_CLASSAPI_BIT|TRACE_CLASSSTATE_BIT)
#else
#  define TRACE_CLASS_BITS      (0)
#endif

#ifdef CONFIG_NSH_USBDEV_TRACETRANSFERS
#  define TRACE_TRANSFER_BITS   (TRACE_OUTREQQUEUED_BIT|TRACE_INREQQUEUED_BIT|TRACE_READ_BIT|\
                                 TRACE_WRITE_BIT|TRACE_COMPLETE_BIT)
#else
#  define TRACE_TRANSFER_BITS   (0)
#endif

#ifdef CONFIG_NSH_USBDEV_TRACECONTROLLER
#  define TRACE_CONTROLLER_BITS (TRACE_EP_BIT|TRACE_DEV_BIT)
#else
#  define TRACE_CONTROLLER_BITS (0)
#endif

#ifdef CONFIG_NSH_USBDEV_TRACEINTERRUPTS
#  define TRACE_INTERRUPT_BITS  (TRACE_INTENTRY_BIT|TRACE_INTDECODE_BIT|TRACE_INTEXIT_BIT)
#else
#  define TRACE_INTERRUPT_BITS  (0)
#endif

#define TRACE_BITSET            (TRACE_INIT_BITS|TRACE_ERROR_BITS|TRACE_CLASS_BITS|\
                                 TRACE_TRANSFER_BITS|TRACE_CONTROLLER_BITS|TRACE_INTERRUPT_BITS)

#endif

/* If Telnet is selected for the NSH console, then we must configure
 * the resources used by the Telnet daemon and by the Telnet clients.
 *
 * CONFIG_NSH_TELNETD_PORT - The telnet daemon will listen on this.
 *   port.  Default: 23
 * CONFIG_NSH_TELNETD_DAEMONPRIO - Priority of the Telnet daemon.
 *   Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_NSH_TELNETD_DAEMONSTACKSIZE - Stack size allocated for the
 *   Telnet daemon. Default: 2048
 * CONFIG_NSH_TELNETD_CLIENTPRIO - Priority of the Telnet client.
 *   Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_NSH_TELNETD_CLIENTSTACKSIZE - Stack size allocated for the
 *   Telnet client. Default: 2048
 * CONFIG_NSH_TELNET_LOGIN - Support a simple Telnet login.
 *
 * If CONFIG_NSH_TELNET_LOGIN is defined, then these additional
 * options may be specified:
 *
 * CONFIG_NSH_TELNET_USERNAME - Login user name.  Default: "admin"
 * CONFIG_NSH_TELNET_PASSWORD - Login password:  Default: "nuttx"
 * CONFIG_NSH_TELNET_FAILCOUNT - Number of login retry attempts.
 *   Default 3.
 */

#ifndef CONFIG_NSH_TELNETD_PORT
#  define CONFIG_NSH_TELNETD_PORT 23
#endif

#ifndef CONFIG_NSH_TELNETD_DAEMONPRIO
#  define CONFIG_NSH_TELNETD_DAEMONPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_NSH_TELNETD_DAEMONSTACKSIZE
#  define CONFIG_NSH_TELNETD_DAEMONSTACKSIZE 2048
#endif

#ifndef CONFIG_NSH_TELNETD_CLIENTPRIO
#  define CONFIG_NSH_TELNETD_CLIENTPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_NSH_TELNETD_CLIENTSTACKSIZE
#  define CONFIG_NSH_TELNETD_CLIENTSTACKSIZE 2048
#endif

#ifdef CONFIG_NSH_TELNET_LOGIN

#  ifndef CONFIG_NSH_TELNET_USERNAME
#    define CONFIG_NSH_TELNET_USERNAME  "admin"
#  endif

#  ifndef CONFIG_NSH_TELNET_PASSWORD
#    define CONFIG_NSH_TELNET_PASSWORD  "nuttx"
#  endif

#  ifndef CONFIG_NSH_TELNET_FAILCOUNT
#    define CONFIG_NSH_TELNET_FAILCOUNT 3
#  endif

#endif /* CONFIG_NSH_TELNET_LOGIN */

/* CONFIG_NSH_MAX_ROUNDTRIP - This is the maximum round trip for a response to
 *   a ICMP ECHO request. It is in units of deciseconds.  The default is 20
 *   (2 seconds).
 */

#ifndef CONFIG_NSH_MAX_ROUNDTRIP
#  define CONFIG_NSH_MAX_ROUNDTRIP 20
#endif

/* Verify support for ROMFS /etc directory support options */

#ifdef CONFIG_NSH_ROMFSETC
#  ifdef CONFIG_DISABLE_MOUNTPOINT
#    error "Mountpoint support is disabled"
#    undef CONFIG_NSH_ROMFSETC
#  endif
#  if CONFIG_NFILE_DESCRIPTORS < 4
#    error "Not enough file descriptors"
#    undef CONFIG_NSH_ROMFSETC
#  endif
#  ifndef CONFIG_FS_ROMFS
#    error "ROMFS support not enabled"
#    undef CONFIG_NSH_ROMFSETC
#  endif
#  ifndef CONFIG_NSH_ROMFSMOUNTPT
#    define CONFIG_NSH_ROMFSMOUNTPT "/etc"
#  endif
#  ifdef CONFIG_NSH_INIT
#    ifndef CONFIG_NSH_INITSCRIPT
#      define CONFIG_NSH_INITSCRIPT "init.d/rcS"
#    endif
#  endif
#  undef NSH_INITPATH
#  define NSH_INITPATH CONFIG_NSH_ROMFSMOUNTPT "/" CONFIG_NSH_INITSCRIPT
#  ifndef CONFIG_NSH_ROMFSDEVNO
#    define CONFIG_NSH_ROMFSDEVNO 0
#  endif
#  ifndef CONFIG_NSH_ROMFSSECTSIZE
#    define CONFIG_NSH_ROMFSSECTSIZE 64
#  endif
#  define NSECTORS(b)        (((b)+CONFIG_NSH_ROMFSSECTSIZE-1)/CONFIG_NSH_ROMFSSECTSIZE)
#  define STR_RAMDEVNO(m)    #m
#  define MKMOUNT_DEVNAME(m) "/dev/ram" STR_RAMDEVNO(m)
#  define MOUNT_DEVNAME      MKMOUNT_DEVNAME(CONFIG_NSH_ROMFSDEVNO)
#else
#  undef CONFIG_NSH_ROMFSMOUNTPT
#  undef CONFIG_NSH_INIT
#  undef CONFIG_NSH_INITSCRIPT
#  undef CONFIG_NSH_ROMFSDEVNO
#  undef CONFIG_NSH_ROMFSSECTSIZE
#endif

/* This is the maximum number of arguments that will be accepted for a
 * command.  Here we attempt to select the smallest number possible depending
 * upon the of commands that are available.  Most commands use six or fewer
 * arguments, but there are a few that require more.
 *
 * This value is also configurable with CONFIG_NSH_MAXARGUMENTS.  This
 * configurability is necessary since there may also be external, "built-in"
 * commands that require more commands than NSH is aware of.
 */

#ifndef CONFIG_NSH_MAXARGUMENTS
#  define CONFIG_NSH_MAXARGUMENTS 6
#endif

#if CONFIG_NSH_MAXARGUMENTS < 11
#  if defined(CONFIG_NET) && !defined(CONFIG_NSH_DISABLE_IFCONFIG)
#    undef CONFIG_NSH_MAXARGUMENTS
#    define CONFIG_NSH_MAXARGUMENTS 11
#  endif
#endif

#if CONFIG_NSH_MAXARGUMENTS < 7
#  if defined(CONFIG_NET_UDP) && CONFIG_NFILE_DESCRIPTORS > 0
#    if !defined(CONFIG_NSH_DISABLE_GET) || !defined(CONFIG_NSH_DISABLE_PUT)
#      undef CONFIG_NSH_MAXARGUMENTS
#      define CONFIG_NSH_MAXARGUMENTS 7
#    endif
# endif
#endif

/* strerror() produces much nicer output but is, however, quite large and
 * will only be used if CONFIG_NSH_STRERROR is defined.  Note that the strerror
 * interface must also have been enabled with CONFIG_LIBC_STRERROR.
 */

#ifndef CONFIG_LIBC_STRERROR
#  undef CONFIG_NSH_STRERROR
#endif

#ifdef CONFIG_NSH_STRERROR
#  define NSH_ERRNO         strerror(errno)
#  define NSH_ERRNO_OF(err) strerror(err)
#else
#  define NSH_ERRNO         (errno)
#  define NSH_ERRNO_OF(err) (err)
#endif

/* Maximum size of one command line (telnet or serial) */

#ifndef CONFIG_NSH_LINELEN
#  define CONFIG_NSH_LINELEN 80
#endif

/* The following two settings are used only in the telnetd interface */

#ifndef CONFIG_NSH_IOBUFFER_SIZE
# define CONFIG_NSH_IOBUFFER_SIZE 512
#endif

/* The maximum number of nested if-then[-else]-fi sequences that
 * are permissable.
 */

#ifndef CONFIG_NSH_NESTDEPTH
# define CONFIG_NSH_NESTDEPTH 3
#endif

/* Define to enable dumping of all input/output buffers */

#undef CONFIG_NSH_TELNETD_DUMPBUFFER
#undef CONFIG_NSH_FULLPATH

/* Make sure that the home directory is defined */

#ifndef CONFIG_LIB_HOMEDIR
# define CONFIG_LIB_HOMEDIR "/"
#endif

/* Stubs used when working directory is not supported */

#if CONFIG_NFILE_DESCRIPTORS <= 0 || defined(CONFIG_DISABLE_ENVIRON)
#  define nsh_getfullpath(v,p) ((char*)(p))
#  define nsh_freefullpath(p)
#endif

/* The size of the I/O buffer may be specified in the
 * configs/<board-name>defconfig file -- provided that it is at least as
 * large as PATH_MAX.
 */

#if CONFIG_NFILE_DESCRIPTORS > 0
#  ifdef CONFIG_NSH_FILEIOSIZE
#    if CONFIG_NSH_FILEIOSIZE > (PATH_MAX + 1)
#      define IOBUFFERSIZE CONFIG_NSH_FILEIOSIZE
#    else
#      define IOBUFFERSIZE (PATH_MAX + 1)
#    endif
#  else
#    define IOBUFFERSIZE 1024
#  endif
# else
#    define IOBUFFERSIZE (PATH_MAX + 1)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum nsh_parser_e
{
   NSH_PARSER_NORMAL = 0,
   NSH_PARSER_IF,
   NSH_PARSER_THEN,
   NSH_PARSER_ELSE
};

struct nsh_state_s
{
  uint8_t   ns_ifcond   : 1; /* Value of command in 'if' statement */
  uint8_t   ns_disabled : 1; /* TRUE: Unconditionally disabled */
  uint8_t   ns_unused   : 4;
  uint8_t   ns_state    : 2; /* Parser state (see enum nsh_parser_e) */
};

struct nsh_parser_s
{
#ifndef CONFIG_NSH_DISABLEBG
  bool    np_bg;       /* true: The last command executed in background */
#endif
  bool    np_redirect; /* true: Output from the last command was re-directed */
  bool    np_fail;     /* true: The last command failed */
#ifndef CONFIG_NSH_DISABLESCRIPT
  uint8_t np_ndx;      /* Current index into np_st[] */
#endif
#ifndef CONFIG_NSH_DISABLEBG
  int     np_nice;     /* "nice" value applied to last background cmd */
#endif

  /* This is a stack of parser state information.  It supports nested
   * execution of commands that span multiple lines (like if-then-else-fi)
   */

#ifndef CONFIG_NSH_DISABLESCRIPT
  struct nsh_state_s np_st[CONFIG_NSH_NESTDEPTH];
#endif
};

struct nsh_vtbl_s; /* Defined in nsh_console.h */
typedef int (*cmd_t)(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const char g_nshgreeting[];
#if defined(CONFIG_NSH_TELNET_LOGIN) && defined(CONFIG_NSH_TELNET)
extern const char g_telnetgreeting[];
extern const char g_userprompt[];
extern const char g_passwordprompt[];
extern const char g_loginsuccess[];
extern const char g_badcredentials[];
extern const char g_loginfailure[];
#endif
extern const char g_nshprompt[];
extern const char g_nshsyntax[];
extern const char g_fmtargrequired[];
extern const char g_fmtarginvalid[];
extern const char g_fmtargrange[];
extern const char g_fmtcmdnotfound[];
extern const char g_fmtnosuch[];
extern const char g_fmttoomanyargs[];
extern const char g_fmtdeepnesting[];
extern const char g_fmtcontext[];
extern const char g_fmtcmdfailed[];
extern const char g_fmtcmdoutofmemory[];
extern const char g_fmtinternalerror[];
#ifndef CONFIG_DISABLE_SIGNALS
extern const char g_fmtsignalrecvd[];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialization */

#ifdef CONFIG_NSH_ROMFSETC
int nsh_romfsetc(void);
#else
#  define nsh_romfsetc() (-ENOSYS)
#endif

#ifdef CONFIG_NET
int nsh_netinit(void);
#else
#  define nsh_netinit() (-ENOSYS)
#endif

#ifdef HAVE_USB_CONSOLE
int nsh_usbconsole(void);
#else
#  define nsh_usbconsole() (-ENOSYS)
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0 && !defined(CONFIG_NSH_DISABLESCRIPT)
int nsh_script(FAR struct nsh_vtbl_s *vtbl, const char *cmd, const char *path);
#endif

/* Architecture-specific initialization */

#ifdef CONFIG_NSH_ARCHINIT
int nsh_archinitialize(void);
#else
#  define nsh_archinitialize() (-ENOSYS)
#endif

/* Message handler */

int nsh_parse(FAR struct nsh_vtbl_s *vtbl, char *cmdline);

/* Application interface */

#ifdef CONFIG_NSH_BUILTIN_APPS
int nsh_builtin(FAR struct nsh_vtbl_s *vtbl, FAR const char *cmd,
                FAR char **argv, FAR const char *redirfile, int oflags);
#endif

#ifdef CONFIG_NSH_FILE_APPS
int nsh_fileapp(FAR struct nsh_vtbl_s *vtbl, FAR const char *cmd,
                FAR char **argv, FAR const char *redirfile, int oflags);
#endif

/* Working directory support */

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
FAR const char *nsh_getcwd(void);
char *nsh_getfullpath(FAR struct nsh_vtbl_s *vtbl, const char *relpath);
void nsh_freefullpath(char *relpath);
#endif

/* Debug */

void nsh_dumpbuffer(FAR struct nsh_vtbl_s *vtbl, const char *msg,
                    const uint8_t *buffer, ssize_t nbytes);

/* USB debug support */

#if defined(CONFIG_USBDEV_TRACE) && defined(HAVE_USB_CONSOLE)
void nsh_usbtrace(void);
#else
#  define nsh_usbtrace()
#endif

/* Shell command handlers */

#ifndef CONFIG_NSH_DISABLE_ECHO
  int cmd_echo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
#ifndef CONFIG_NSH_DISABLE_EXEC
  int cmd_exec(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
#ifndef CONFIG_NSH_DISABLE_MB
  int cmd_mb(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
#ifndef CONFIG_NSH_DISABLE_MH
  int cmd_mh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
#ifndef CONFIG_NSH_DISABLE_MW
  int cmd_mw(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
#ifndef CONFIG_NSH_DISABLE_FREE
  int cmd_free(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
#ifndef CONFIG_NSH_DISABLE_PS
  int cmd_ps(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
#ifndef CONFIG_NSH_DISABLE_XD
  int cmd_xd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
  
#if !defined(CONFIG_NSH_DISABLESCRIPT) && !defined(CONFIG_NSH_DISABLE_TEST)
  int cmd_test(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
  int cmd_lbracket(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif

#ifndef CONFIG_DISABLE_CLOCK
#  if defined (CONFIG_RTC) && !defined(CONFIG_NSH_DISABLE_DATE)
   int cmd_date(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0
#  ifndef CONFIG_NSH_DISABLE_CAT
      int cmd_cat(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_CP
      int cmd_cp(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_DD
      int cmd_dd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_HEXDUMP
      int cmd_hexdump(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
#  ifndef CONFIG_NSH_DISABLE_LS
      int cmd_ls(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  if defined(CONFIG_SYSLOG) && defined(CONFIG_RAMLOG_SYSLOG) && !defined(CONFIG_NSH_DISABLE_DMESG)
      int cmd_dmesg(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  if CONFIG_NFILE_STREAMS > 0 && !defined(CONFIG_NSH_DISABLESCRIPT)
#     ifndef CONFIG_NSH_DISABLE_SH
       int cmd_sh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#     endif
# endif  /* CONFIG_NFILE_STREAMS && !CONFIG_NSH_DISABLESCRIPT */
# ifndef CONFIG_DISABLE_MOUNTPOINT
#   ifndef CONFIG_NSH_DISABLE_LOSETUP
       int cmd_losetup(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
#   ifndef CONFIG_NSH_DISABLE_MKFIFO
       int cmd_mkfifo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
#   ifdef CONFIG_FS_READABLE
#     ifndef CONFIG_NSH_DISABLE_DF
         int cmd_df(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#     endif
#     ifndef CONFIG_NSH_DISABLE_MOUNT
         int cmd_mount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#     endif
#     ifndef CONFIG_NSH_DISABLE_UMOUNT
         int cmd_umount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#     endif
#     ifdef CONFIG_FS_WRITABLE
#       ifndef CONFIG_NSH_DISABLE_MKDIR
           int cmd_mkdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#       endif
#       ifndef CONFIG_NSH_DISABLE_MKRD
           int cmd_mkrd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#       endif
#       ifndef CONFIG_NSH_DISABLE_MV
           int cmd_mv(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#       endif
#       ifndef CONFIG_NSH_DISABLE_RM
           int cmd_rm(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#       endif
#       ifndef CONFIG_NSH_DISABLE_RMDIR
           int cmd_rmdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#       endif
#     endif /* CONFIG_FS_WRITABLE */
#   endif /* CONFIG_FS_READABLE */
#   ifdef CONFIG_FS_FAT
#     ifndef CONFIG_NSH_DISABLE_MKFATFS
         int cmd_mkfatfs(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#     endif
#   endif /* CONFIG_FS_FAT */
# endif /* !CONFIG_DISABLE_MOUNTPOINT */
# if !defined(CONFIG_DISABLE_ENVIRON)
#   ifndef CONFIG_NSH_DISABLE_CD
       int cmd_cd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
#   ifndef CONFIG_NSH_DISABLE_PWD
       int cmd_pwd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
# endif /* !CONFIG_DISABLE_MOUNTPOINT */
#endif /* CONFIG_NFILE_DESCRIPTORS */

#if defined(CONFIG_NET)
#  ifndef CONFIG_NSH_DISABLE_IFCONFIG
      int cmd_ifconfig(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_IFUPDOWN
      int cmd_ifup(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
      int cmd_ifdown(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#if defined(CONFIG_NET_UDP) && CONFIG_NFILE_DESCRIPTORS > 0
#  ifndef CONFIG_NSH_DISABLE_GET
      int cmd_get(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_PUT
      int cmd_put(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif
#if defined(CONFIG_NET_TCP) && CONFIG_NFILE_DESCRIPTORS > 0
#  ifndef CONFIG_NSH_DISABLE_WGET
      int cmd_wget(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif
#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING) && \
   !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_DISABLE_SIGNALS)
#  ifndef CONFIG_NSH_DISABLE_PING
      int cmd_ping(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif
#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && \
    defined(CONFIG_FS_READABLE) && defined(CONFIG_NFS)
#  ifndef CONFIG_NSH_DISABLE_NFSMOUNT
      int cmd_nfsmount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif
#endif

#ifndef CONFIG_DISABLE_ENVIRON
#  ifndef CONFIG_NSH_DISABLE_SET
      int cmd_set(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_UNSET
      int cmd_unset(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif /* CONFIG_DISABLE_ENVIRON */

#ifndef CONFIG_DISABLE_SIGNALS
#  ifndef CONFIG_NSH_DISABLE_KILL
      int cmd_kill(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_SLEEP
      int cmd_sleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_USLEEP
      int cmd_usleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif /* CONFIG_DISABLE_SIGNALS */

#if defined(CONFIG_NETUTILS_CODECS) && defined(CONFIG_CODECS_BASE64)
#  ifndef CONFIG_NSH_DISABLE_BASE64DEC
      int cmd_base64decode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_BASE64ENC
      int cmd_base64encode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif

#if defined(CONFIG_NETUTILS_CODECS) && defined(CONFIG_CODECS_HASH_MD5)
#  ifndef CONFIG_NSH_DISABLE_MD5
      int cmd_md5(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif

#if defined(CONFIG_NETUTILS_CODECS) && defined(CONFIG_CODECS_URLCODE)
#  ifndef CONFIG_NSH_DISABLE_URLDECODE
      int cmd_urlencode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#  ifndef CONFIG_NSH_DISABLE_URLENCODE
      int cmd_urldecode(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
#endif

#endif /* __APPS_NSHLIB_NSH_H */
