/****************************************************************************
 * apps/nshlib/nsh.h
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#ifdef CONFIG_NSH_CONSOLE
# include <stdio.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The telnetd interface and background commands require pthread support */

#ifdef CONFIG_DISABLE_PTHREAD
#  undef CONFIG_NSH_TELNET
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
#  if defined(CONFIG_USBSER) && defined(CONFIG_USBSER_CONSOLE)
#    define HAVE_USB_CONSOLE 1
#  elif defined(CONFIG_CDCSER) && defined(CONFIG_CDCSER_CONSOLE)
#    define HAVE_USB_CONSOLE 1
#  endif
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

/* This is the maximum number of arguments that will be accepted for a command */

#define NSH_MAX_ARGUMENTS 6

/* strerror() produces much nicer output but is, however, quite large and
 * will only be used if CONFIG_NSH_STRERROR is defined.
 */

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

/* As threads are created to handle each request, a stack must be allocated
 * for the thread.  Use a default if the user provided no stacksize.
 */

#ifndef CONFIG_NSH_STACKSIZE
# define CONFIG_NSH_STACKSIZE 4096
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

/* Method access macros */

#define nsh_clone(v)           (v)->clone(v)
#define nsh_release(v)         (v)->release(v)
#define nsh_write(v,b,n)       (v)->write(v,b,n)
#define nsh_linebuffer(v)      (v)->linebuffer(v)
#define nsh_redirect(v,f,s)    (v)->redirect(v,f,s)
#define nsh_undirect(v,s)      (v)->undirect(v,s)
#define nsh_exit(v)            (v)->exit(v)

#ifdef CONFIG_CPP_HAVE_VARARGS
# define nsh_output(v, fmt...) (v)->output(v, ##fmt)
#else
# define nsh_output            vtbl->output
#endif

/* Size of info to be saved in call to nsh_redirect */

#define SAVE_SIZE (sizeof(int) + sizeof(FILE*) + sizeof(bool))

/* Stubs used when working directory is not supported */

#if CONFIG_NFILE_DESCRIPTORS <= 0 || defined(CONFIG_DISABLE_ENVIRON)
#  define nsh_getfullpath(v,p) ((char*)(p))
#  define nsh_freefullpath(p)
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

struct nsh_vtbl_s
{
  /* This function pointers are "hooks" into the front end logic to
   * handle things like output of command results, redirection, etc.
   * -- all of which must be done in a way that is unique to the nature
   * of the front end.
   */

#ifndef CONFIG_NSH_DISABLEBG
  FAR struct nsh_vtbl_s *(*clone)(FAR struct nsh_vtbl_s *vtbl);
  void (*addref)(FAR struct nsh_vtbl_s *vtbl);
  void (*release)(FAR struct nsh_vtbl_s *vtbl);
#endif
  ssize_t (*write)(FAR struct nsh_vtbl_s *vtbl, FAR const void *buffer, size_t nbytes);
  int (*output)(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);
  FAR char *(*linebuffer)(FAR struct nsh_vtbl_s *vtbl);
  void (*redirect)(FAR struct nsh_vtbl_s *vtbl, int fd, FAR uint8_t *save);
  void (*undirect)(FAR struct nsh_vtbl_s *vtbl, FAR uint8_t *save);
  void (*exit)(FAR struct nsh_vtbl_s *vtbl);

  /* Parser state data */

  struct nsh_parser_s np;
};

typedef int (*cmd_t)(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const char g_nshgreeting[];
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
int nsh_execapp(FAR struct nsh_vtbl_s *vtbl, FAR const char *cmd, FAR char **argv);
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
#  ifndef CONFIG_NSH_DISABLE_LS
      int cmd_ls(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#  endif
# if CONFIG_NFILE_STREAMS > 0 && !defined(CONFIG_NSH_DISABLESCRIPT)
#   ifndef CONFIG_NSH_DISABLE_SH
       int cmd_sh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
# endif  /* CONFIG_NFILE_STREAMS && !CONFIG_NSH_DISABLESCRIPT */
# ifndef CONFIG_DISABLE_MOUNTPOINT
#   ifndef CONFIG_NSH_DISABLE_LOSETUP
       int cmd_losetup(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
#   ifndef CONFIG_NSH_DISABLE_MKFIFO
       int cmd_mkfifo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#   endif
#   ifdef CONFIG_FS_READABLE
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

#endif /* __APPS_NSHLIB_NSH_H */
