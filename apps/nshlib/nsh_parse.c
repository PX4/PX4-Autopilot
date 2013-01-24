/****************************************************************************
 * apps/nshlib/nsh_parse.c
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/version.h>

#ifndef CONFIG_NSH_DISABLEBG
#  include <pthread.h>
#endif

#ifdef CONFIG_NSH_BUILTIN_APPS
#  include <nuttx/binfmt/builtin.h>
#endif

#include <apps/nsh.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Argument list size
 *
 *   argv[0]:      The command name.
 *   argv[1]:      The beginning of argument (up to CONFIG_NSH_MAXARGUMENTS)
 *   argv[argc-3]: Possibly '>' or '>>'
 *   argv[argc-2]: Possibly <file>
 *   argv[argc-1]: Possibly '&' (if pthreads are enabled)
 *   argv[argc]:   NULL terminating pointer
 *
 * Maximum size is CONFIG_NSH_MAXARGUMENTS+5
 */

#ifndef CONFIG_NSH_DISABLEBG
#  define MAX_ARGV_ENTRIES (CONFIG_NSH_MAXARGUMENTS+5)
#else
#  define MAX_ARGV_ENTRIES (CONFIG_NSH_MAXARGUMENTS+4)
#endif

/* Help command summary layout */

#define MAX_CMDLEN    12
#define CMDS_PER_LINE 6

#define NUM_CMDS      ((sizeof(g_cmdmap)/sizeof(struct cmdmap_s)) - 1)
#define NUM_CMD_ROWS  ((NUM_CMDS + (CMDS_PER_LINE-1)) / CMDS_PER_LINE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cmdmap_s
{
  const char *cmd;        /* Name of the command */
  cmd_t       handler;    /* Function that handles the command */
  uint8_t     minargs;    /* Minimum number of arguments (including command) */
  uint8_t     maxargs;    /* Maximum number of arguments (including command) */
  const char *usage;      /* Usage instructions for 'help' command */
};

#ifndef CONFIG_NSH_DISABLEBG
struct cmdarg_s
{
  FAR struct nsh_vtbl_s *vtbl;      /* For front-end interaction */
  int fd;                           /* FD for output redirection */
  int argc;                         /* Number of arguments in argv */
  FAR char *argv[MAX_ARGV_ENTRIES]; /* Argument list */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_HELP
static int  cmd_help(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif

#ifndef CONFIG_NSH_DISABLE_EXIT
static int  cmd_exit(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);
#endif
static int  cmd_unrecognized(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_delim[]      = " \t\n";
static const char g_redirect1[]  = ">";
static const char g_redirect2[]  = ">>";
static const char g_exitstatus[] = "$?";
static const char g_success[]    = "0";
static const char g_failure[]    = "1";

static const struct cmdmap_s g_cmdmap[] =
{
#if !defined(CONFIG_NSH_DISABLESCRIPT) && !defined(CONFIG_NSH_DISABLE_TEST)
  { "[",        cmd_lbracket, 4, CONFIG_NSH_MAXARGUMENTS, "<expression> ]" },
#endif

#ifndef CONFIG_NSH_DISABLE_HELP
  { "?",        cmd_help,     1, 1, NULL },
#endif

#if defined(CONFIG_NETUTILS_CODECS) && defined(CONFIG_CODECS_BASE64)
#  ifndef CONFIG_NSH_DISABLE_BASE64DEC
  { "base64dec", cmd_base64decode, 2, 4, "[-w] [-f] <string or filepath>" },
#  endif
#  ifndef CONFIG_NSH_DISABLE_BASE64ENC
  { "base64enc", cmd_base64encode, 2, 4, "[-w] [-f] <string or filepath>" },
#  endif
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0
# ifndef CONFIG_NSH_DISABLE_CAT
  { "cat",      cmd_cat,      2, CONFIG_NSH_MAXARGUMENTS, "<path> [<path> [<path> ...]]" },
# endif
#ifndef CONFIG_DISABLE_ENVIRON
# ifndef CONFIG_NSH_DISABLE_CD
  { "cd",       cmd_cd,       1, 2, "[<dir-path>|-|~|..]" },
# endif
#endif
# ifndef CONFIG_NSH_DISABLE_CP
  { "cp",       cmd_cp,       3, 3, "<source-path> <dest-path>" },
# endif
#endif

#if defined (CONFIG_RTC) && !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_NSH_DISABLE_DATE)
  { "date",     cmd_date,     1, 3, "[-s \"MMM DD HH:MM:SS YYYY\"]" },
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_NSH_DISABLE_DD)
  { "dd",       cmd_dd,       3, 6, "if=<infile> of=<outfile> [bs=<sectsize>] [count=<sectors>] [skip=<sectors>]" },
# endif

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    defined(CONFIG_FS_READABLE) && !defined(CONFIG_NSH_DISABLE_DF)
  { "df",       cmd_df,       1, 1, NULL },
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_SYSLOG) && \
    defined(CONFIG_RAMLOG_SYSLOG) && !defined(CONFIG_NSH_DISABLE_DMESG)
  { "dmesg",    cmd_dmesg,    1, 1, NULL },
#endif

#ifndef CONFIG_NSH_DISABLE_ECHO
# ifndef CONFIG_DISABLE_ENVIRON
  { "echo",     cmd_echo,     0, CONFIG_NSH_MAXARGUMENTS, "[<string|$name> [<string|$name>...]]" },
# else
  { "echo",     cmd_echo,     0, CONFIG_NSH_MAXARGUMENTS, "[<string> [<string>...]]" },
# endif
#endif

#ifndef CONFIG_NSH_DISABLE_EXEC
  { "exec",     cmd_exec,     2, 3, "<hex-address>" },
#endif
#ifndef CONFIG_NSH_DISABLE_EXIT
  { "exit",     cmd_exit,     1, 1, NULL },
#endif

#ifndef CONFIG_NSH_DISABLE_FREE
  { "free",     cmd_free,     1, 1, NULL },
#endif

#if defined(CONFIG_NET_UDP) && CONFIG_NFILE_DESCRIPTORS > 0
# ifndef CONFIG_NSH_DISABLE_GET
  { "get",      cmd_get,      4, 7, "[-b|-n] [-f <local-path>] -h <ip-address> <remote-path>" },
# endif
#endif

#ifndef CONFIG_NSH_DISABLE_HELP
# ifdef CONFIG_NSH_HELP_TERSE
  { "help",     cmd_help,     1, 2, "[<cmd>]" },
#else
  { "help",     cmd_help,     1, 3, "[-v] [<cmd>]" },
# endif
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifndef CONFIG_NSH_DISABLE_HEXDUMP
  { "hexdump",  cmd_hexdump,  2, 2, "<file or device>" },
#endif
#endif

#ifdef CONFIG_NET
# ifndef CONFIG_NSH_DISABLE_IFCONFIG
  { "ifconfig", cmd_ifconfig, 1, 11, "[nic_name [<ip-address>|dhcp]] [dr|gw|gateway <dr-address>] [netmask <net-mask>] [dns <dns-address>] [hw <hw-mac>]" },
# endif
# ifndef CONFIG_NSH_DISABLE_IFUPDOWN
  { "ifdown",   cmd_ifdown,   2, 2,  "<nic_name>" },
  { "ifup",     cmd_ifup,     2, 2,  "<nic_name>" },
# endif
#endif

#ifndef CONFIG_DISABLE_SIGNALS
# ifndef CONFIG_NSH_DISABLE_KILL
  { "kill",     cmd_kill,     3, 3, "-<signal> <pid>" },
# endif
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
# ifndef CONFIG_NSH_DISABLE_LOSETUP
  { "losetup",   cmd_losetup, 3, 6, "[-d <dev-path>] | [[-o <offset>] [-r] <dev-path> <file-path>]" },
# endif
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0
# ifndef CONFIG_NSH_DISABLE_LS
  { "ls",       cmd_ls,       1, 5, "[-lRs] <dir-path>" },
# endif
#endif

#ifndef CONFIG_NSH_DISABLE_MB
  { "mb",       cmd_mb,       2, 3, "<hex-address>[=<hex-value>][ <hex-byte-count>]" },
#endif

#if defined(CONFIG_NETUTILS_CODECS) && defined(CONFIG_CODECS_HASH_MD5)
#  ifndef CONFIG_NSH_DISABLE_MD5
  { "md5",      cmd_md5,      2, 3, "[-f] <string or filepath>" },
#  endif
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
# ifndef CONFIG_NSH_DISABLE_MKDIR
  { "mkdir",    cmd_mkdir,    2, 2, "<path>" },
# endif
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_FAT)
# ifndef CONFIG_NSH_DISABLE_MKFATFS
  { "mkfatfs",  cmd_mkfatfs,  2, 2, "<path>" },
# endif
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
# ifndef CONFIG_NSH_DISABLE_MKFIFO
  { "mkfifo",   cmd_mkfifo,   2, 2, "<path>" },
# endif
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
# ifndef CONFIG_NSH_DISABLE_MKRD
  { "mkrd",     cmd_mkrd,     2, 6, "[-m <minor>] [-s <sector-size>] <nsectors>" },
# endif
#endif

#ifndef CONFIG_NSH_DISABLE_MH
  { "mh",       cmd_mh,       2, 3, "<hex-address>[=<hex-value>][ <hex-byte-count>]" },
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_READABLE)
# ifndef CONFIG_NSH_DISABLE_MOUNT
  { "mount",    cmd_mount,    1, 5, "[-t <fstype> [<block-device>] <mount-point>]" },
# endif
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
#  ifndef CONFIG_NSH_DISABLE_MV
  { "mv",       cmd_mv,       3, 3, "<old-path> <new-path>" },
#  endif
#endif

#ifndef CONFIG_NSH_DISABLE_MW
  { "mw",       cmd_mw,       2, 3, "<hex-address>[=<hex-value>][ <hex-byte-count>]" },
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && \
    defined(CONFIG_NET) && defined(CONFIG_NFS)
#  ifndef CONFIG_NSH_DISABLE_NFSMOUNT
  { "nfsmount", cmd_nfsmount, 4, 4, "<server-address> <mount-point> <remote-path>" },
#  endif
#endif

#if defined(CONFIG_NET) && defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING) && \
   !defined(CONFIG_DISABLE_CLOCK) && !defined(CONFIG_DISABLE_SIGNALS)
# ifndef CONFIG_NSH_DISABLE_PING
  { "ping",     cmd_ping,     2, 6, "[-c <count>] [-i <interval>] <ip-address>" },
# endif
#endif

#ifndef CONFIG_NSH_DISABLE_PS
  { "ps",       cmd_ps,       1, 1, NULL },
#endif

#if defined(CONFIG_NET_UDP) && CONFIG_NFILE_DESCRIPTORS > 0
# ifndef CONFIG_NSH_DISABLE_PUT
  { "put",      cmd_put,      4, 7, "[-b|-n] [-f <remote-path>] -h <ip-address> <local-path>" },
# endif
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_ENVIRON)
# ifndef CONFIG_NSH_DISABLE_PWD
  { "pwd",      cmd_pwd,      1, 1, NULL },
# endif
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
# ifndef CONFIG_NSH_DISABLE_RM
  { "rm",       cmd_rm,       2, 2, "<file-path>" },
# endif
# ifndef CONFIG_NSH_DISABLE_RMDIR
  { "rmdir",    cmd_rmdir,    2, 2, "<dir-path>" },
# endif
#endif

#ifndef CONFIG_DISABLE_ENVIRON
# ifndef CONFIG_NSH_DISABLE_SET
  { "set",      cmd_set,      3, 3, "<name> <value>" },
# endif
#endif

#if  CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0 && !defined(CONFIG_NSH_DISABLESCRIPT)
# ifndef CONFIG_NSH_DISABLE_SH
  { "sh",       cmd_sh,       2, 2, "<script-path>" },
# endif
#endif

#ifndef CONFIG_DISABLE_SIGNALS
# ifndef CONFIG_NSH_DISABLE_SLEEP
  { "sleep",    cmd_sleep,    2, 2, "<sec>" },
# endif
#endif

#if !defined(CONFIG_NSH_DISABLESCRIPT) && !defined(CONFIG_NSH_DISABLE_TEST)
  { "test",     cmd_test,     3, CONFIG_NSH_MAXARGUMENTS, "<expression>" },
#endif

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_READABLE)
# ifndef CONFIG_NSH_DISABLE_UMOUNT
  { "umount",   cmd_umount,   2, 2, "<dir-path>" },
# endif
#endif

#ifndef CONFIG_DISABLE_ENVIRON
# ifndef CONFIG_NSH_DISABLE_UNSET
  { "unset",    cmd_unset,    2, 2, "<name>" },
# endif
#endif

#if defined(CONFIG_NETUTILS_CODECS) && defined(CONFIG_CODECS_URLCODE)
#  ifndef CONFIG_NSH_DISABLE_URLDECODE
  { "urldecode", cmd_urldecode, 2, 3, "[-f] <string or filepath>" },
#  endif
#  ifndef CONFIG_NSH_DISABLE_URLENCODE
  { "urlencode", cmd_urlencode, 2, 3, "[-f] <string or filepath>" },
#  endif
#endif

#ifndef CONFIG_DISABLE_SIGNALS
# ifndef CONFIG_NSH_DISABLE_USLEEP
  { "usleep",   cmd_usleep,   2, 2, "<usec>" },
# endif
#endif

#if defined(CONFIG_NET_TCP) && CONFIG_NFILE_DESCRIPTORS > 0
# ifndef CONFIG_NSH_DISABLE_WGET
  { "wget",     cmd_wget,     2, 4, "[-o <local-path>] <url>" },
# endif
#endif

#ifndef CONFIG_NSH_DISABLE_XD
  { "xd",       cmd_xd,       3, 3, "<hex-address> <byte-count>" },
#endif

  { NULL,       NULL,         1, 1, NULL }
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If NuttX versioning information is available, Include that information
 * in the NSH greeting.
 */

#if CONFIG_VERSION_MAJOR != 0 || CONFIG_VERSION_MINOR != 0
const char g_nshgreeting[]       = "\nNuttShell (NSH) NuttX-" CONFIG_VERSION_STRING "\n";
#else
const char g_nshgreeting[]       = "\nNuttShell (NSH)\n";
#endif

/* Telnet login prompts */

#if defined(CONFIG_NSH_TELNET_LOGIN) && defined(CONFIG_NSH_TELNET)
const char g_telnetgreeting[]    = "\nWelcome to NuttShell(NSH) Telnet Server...\n";
const char g_userprompt[]        = "login: ";
const char g_passwordprompt[]    = "password: ";
const char g_loginsuccess[]      = "\nUser Logged-in!\n";
const char g_badcredentials[]    = "\nInvalid username or password\n";
const char g_loginfailure[]      = "Login failed!\n";
#endif

/* The NSH prompt */

const char g_nshprompt[]         = "nsh> ";

/* Common, message formats */

const char g_nshsyntax[]         = "nsh: %s: syntax error\n";
const char g_fmtargrequired[]    = "nsh: %s: missing required argument(s)\n";
const char g_fmtarginvalid[]     = "nsh: %s: argument invalid\n";
const char g_fmtargrange[]       = "nsh: %s: value out of range\n";
const char g_fmtcmdnotfound[]    = "nsh: %s: command not found\n";
const char g_fmtnosuch[]         = "nsh: %s: no such %s: %s\n";
const char g_fmttoomanyargs[]    = "nsh: %s: too many arguments\n";
const char g_fmtdeepnesting[]    = "nsh: %s: nesting too deep\n";
const char g_fmtcontext[]        = "nsh: %s: not valid in this context\n";
#ifdef CONFIG_NSH_STRERROR
const char g_fmtcmdfailed[]      = "nsh: %s: %s failed: %s\n";
#else
const char g_fmtcmdfailed[]      = "nsh: %s: %s failed: %d\n";
#endif
const char g_fmtcmdoutofmemory[] = "nsh: %s: out of memory\n";
const char g_fmtinternalerror[]  = "nsh: %s: Internal error\n";
#ifndef CONFIG_DISABLE_SIGNALS
const char g_fmtsignalrecvd[]    = "nsh: %s: Interrupted by signal\n";
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: help_cmdlist
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_HELP
static inline void help_cmdlist(FAR struct nsh_vtbl_s *vtbl)
{
  int i;
  int j;
  int k;

  /* Print the command name in NUM_CMD_ROWS rows with CMDS_PER_LINE commands
   * on each line.
   */

  for (i = 0; i < NUM_CMD_ROWS; i++)
    {
      nsh_output(vtbl, "  ");
      for (j = 0, k = i; j < CMDS_PER_LINE && k < NUM_CMDS; j++, k += NUM_CMD_ROWS)
        {
          nsh_output(vtbl, "%-12s", g_cmdmap[k].cmd);
        }

      nsh_output(vtbl, "\n");
    }
}
#endif

/****************************************************************************
 * Name: help_usage
 ****************************************************************************/

#if !defined(CONFIG_NSH_DISABLE_HELP) && !defined(CONFIG_NSH_HELP_TERSE)
static inline void help_usage(FAR struct nsh_vtbl_s *vtbl)
{
  nsh_output(vtbl, "NSH command forms:\n");
#ifndef CONFIG_NSH_DISABLEBG
  nsh_output(vtbl, "  [nice [-d <niceness>>]] <cmd> [> <file>|>> <file>] [&]\n");
#else
  nsh_output(vtbl, "  <cmd> [> <file>|>> <file>]\n");
#endif
#ifndef CONFIG_NSH_DISABLESCRIPT
  nsh_output(vtbl, "OR\n");
  nsh_output(vtbl, "  if <cmd>\n");
  nsh_output(vtbl, "  then\n");
  nsh_output(vtbl, "    [sequence of <cmd>]\n");
  nsh_output(vtbl, "  else\n");
  nsh_output(vtbl, "    [sequence of <cmd>]\n");
  nsh_output(vtbl, "  fi\n\n");
#endif
}
#endif

/****************************************************************************
 * Name: help_showcmd
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_HELP
static void help_showcmd(FAR struct nsh_vtbl_s *vtbl,
                         FAR const struct cmdmap_s *cmdmap)
{
  if (cmdmap->usage)
    {
      nsh_output(vtbl, "  %s %s\n", cmdmap->cmd, cmdmap->usage);
    }
   else
    {
      nsh_output(vtbl, "  %s\n", cmdmap->cmd);
    }
}
#endif

/****************************************************************************
 * Name: help_cmd
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_HELP
static int help_cmd(FAR struct nsh_vtbl_s *vtbl, FAR const char *cmd)
{
  FAR const struct cmdmap_s *cmdmap;

  /* Find the command in the command table */

  for (cmdmap = g_cmdmap; cmdmap->cmd; cmdmap++)
    {
      /* Is this the one we are looking for? */

      if (strcmp(cmdmap->cmd, cmd) == 0)
        {
          /* Yes... show it */

          nsh_output(vtbl, "%s usage:", cmd);
          help_showcmd(vtbl, cmdmap);
          return OK;
        }
    }

  nsh_output(vtbl, g_fmtcmdnotfound, cmd);
  return ERROR;
}
#endif

/****************************************************************************
 * Name: help_allcmds
 ****************************************************************************/

#if !defined(CONFIG_NSH_DISABLE_HELP) && !defined(CONFIG_NSH_HELP_TERSE)
static inline void help_allcmds(FAR struct nsh_vtbl_s *vtbl)
{
  FAR const struct cmdmap_s *cmdmap;

  /* Show all of the commands in the command table */

  for (cmdmap = g_cmdmap; cmdmap->cmd; cmdmap++)
    {
      help_showcmd(vtbl, cmdmap);
    }
}
#endif

/****************************************************************************
 * Name: help_builtins
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_HELP
static inline void help_builtins(FAR struct nsh_vtbl_s *vtbl)
{
#ifdef CONFIG_NSH_BUILTIN_APPS
  FAR const char *name;
  int i;

  /* List the set of available built-in commands */

  nsh_output(vtbl, "\nBuiltin Apps:\n");
  for (i = 0; (name = builtin_getname(i)) != NULL; i++)
    {
      nsh_output(vtbl, "  %s\n", name);
    }
#endif
}
#endif

/****************************************************************************
 * Name: cmd_help
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_HELP
static int cmd_help(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  FAR const char *cmd = NULL;
#ifndef CONFIG_NSH_HELP_TERSE
  bool verbose = false;
  int i;
#endif

  /* The command may be followed by a verbose option */

#ifndef CONFIG_NSH_HELP_TERSE
  i = 1;
  if (argc > i)
    {
      if (strcmp(argv[i], "-v") == 0)
        {
          verbose = true;
          i++;
        }
    }

  /* The command line may end with a command name */

  if (argc > i)
    {
      cmd = argv[i];
    }

  /* Show the generic usage if verbose is requested */

  if (verbose)
    {
      help_usage(vtbl);
    }
#else
  if (argc > 1)
    {
      cmd = argv[1];
    }
#endif

  /* Are we showing help on a single command? */

  if (cmd)
    {
      /* Yes.. show the single command */

      help_cmd(vtbl, cmd);
    }
  else
    {
       /* In verbose mode, show detailed help for all commands */

#ifndef CONFIG_NSH_HELP_TERSE
      if (verbose)
        {
          nsh_output(vtbl, "Where <cmd> is one of:\n");
          help_allcmds(vtbl);
        }

      /* Otherwise, just show the list of command names */

      else
#endif
        {
          help_cmd(vtbl, "help");
          nsh_output(vtbl, "\n");
          help_cmdlist(vtbl);
        }

      /* And show the list of built-in applications */

      help_builtins(vtbl);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: cmd_unrecognized
 ****************************************************************************/

static int cmd_unrecognized(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_output(vtbl, g_fmtcmdnotfound, argv[0]);
  return ERROR;
}

/****************************************************************************
 * Name: cmd_exit
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_EXIT
static int cmd_exit(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_exit(vtbl, 0);
  return OK;
}
#endif

/****************************************************************************
 * Name: nsh_execute
 *
 * Description:
 *   Execute the command in argv[0]
 *
 * Returned Value:
 *   -1 (ERRROR) if the command was unsuccessful
 *    0 (OK)     if the command was successful
 *
 ****************************************************************************/

static int nsh_execute(FAR struct nsh_vtbl_s *vtbl, int argc, char *argv[])
{
  const struct cmdmap_s *cmdmap;
  const char            *cmd;
  cmd_t                  handler = cmd_unrecognized;
  int                    ret;

  /* The form of argv is:
   *
   * argv[0]:      The command name.  This is argv[0] when the arguments
   *               are, finally, received by the command vtblr
   * argv[1]:      The beginning of argument (up to CONFIG_NSH_MAXARGUMENTS)
   * argv[argc]:   NULL terminating pointer
   */

  cmd = argv[0];

  /* See if the command is one that we understand */

  for (cmdmap = g_cmdmap; cmdmap->cmd; cmdmap++)
    {
      if (strcmp(cmdmap->cmd, cmd) == 0)
        {
          /* Check if a valid number of arguments was provided.  We
           * do this simple, imperfect checking here so that it does
           * not have to be performed in each command.
           */

          if (argc < cmdmap->minargs)
            {
              /* Fewer than the minimum number were provided */

              nsh_output(vtbl, g_fmtargrequired, cmd);
              return ERROR;
            }
          else if (argc > cmdmap->maxargs)
            {
              /* More than the maximum number were provided */

              nsh_output(vtbl, g_fmttoomanyargs, cmd);
              return ERROR;
            }
          else
            {
              /* A valid number of arguments were provided (this does
               * not mean they are right).
               */

              handler = cmdmap->handler;
              break;
            }
        }
    }

   ret = handler(vtbl, argc, argv);
   return ret;
}

/****************************************************************************
 * Name: nsh_releaseargs
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static void nsh_releaseargs(struct cmdarg_s *arg)
{
  FAR struct nsh_vtbl_s *vtbl = arg->vtbl;
  int i;

  /* If the output was redirected, then file descriptor should
   * be closed.  The created task has its one, independent copy of
   * the file descriptor
   */

  if (vtbl->np.np_redirect)
    {
      (void)close(arg->fd);
    }

  /* Released the cloned vtbl instance */

  nsh_release(vtbl);

  /* Release the cloned args */

  for (i = 0; i < arg->argc; i++)
    {
      free(arg->argv[i]);
    }
  free(arg);
}
#endif

/****************************************************************************
 * Name: nsh_child
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static pthread_addr_t nsh_child(pthread_addr_t arg)
{
  struct cmdarg_s *carg = (struct cmdarg_s *)arg;
  int ret;

  dbg("BG %s\n", carg->argv[0]);

  /* Execute the specified command on the child thread */

  ret = nsh_execute(carg->vtbl, carg->argc, carg->argv);

  /* Released the cloned arguments */

  dbg("BG %s complete\n", carg->argv[0]);
  nsh_releaseargs(carg);
  return (void*)ret;
}
#endif

/****************************************************************************
 * Name: nsh_cloneargs
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static inline struct cmdarg_s *nsh_cloneargs(FAR struct nsh_vtbl_s *vtbl,
                                             int fd, int argc, char *argv[])
{
  struct cmdarg_s *ret = (struct cmdarg_s *)zalloc(sizeof(struct cmdarg_s));
  int i;

  if (ret)
    {
      ret->vtbl = vtbl;
      ret->fd   = fd;
      ret->argc = argc;

      for (i = 0; i < argc; i++)
        {
          ret->argv[i] = strdup(argv[i]);
        }
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: nsh_argument
 ****************************************************************************/

char *nsh_argument(FAR struct nsh_vtbl_s *vtbl, char **saveptr)
{
  char *pbegin = *saveptr;
  char *pend   = NULL;
  const char *term;
#ifndef CONFIG_DISABLE_ENVIRON
  bool quoted = false;
#endif

  /* Find the beginning of the next token */

  for (;
       *pbegin && strchr(g_delim, *pbegin) != NULL;
       pbegin++);

  /* If we are at the end of the string with nothing
   * but delimiters found, then return NULL.
   */

  if (!*pbegin)
    {
      return NULL;
    }

  /* Does the token begin with '>' -- redirection of output? */

  if (*pbegin == '>')
    {
      /* Yes.. does it begin with ">>"? */

      if (*(pbegin + 1) == '>')
        {
          *saveptr = pbegin + 2;
          pbegin   = (char*)g_redirect2;
        }
      else
        {
          *saveptr = pbegin + 1;
          pbegin   = (char*)g_redirect1;
        }
    }

  /* Does the token begin with '#' -- comment */

  else if (*pbegin == '#')
    {
      /* Return NULL meaning that we are at the end of the line */

      *saveptr = pbegin;
      pbegin   = NULL;
    }
  else
    {
      /* Otherwise, we are going to have to parse to find the end of
       * the token.  Does the token begin with '"'?
       */

      if (*pbegin == '"')
        {
          /* Yes.. then only another '"' can terminate the string */

          pbegin++;
          term = "\"";
#ifndef CONFIG_DISABLE_ENVIRON
          quoted = true;
#endif
        }
      else
        {
          /* No, then any of the usual terminators will terminate the argument */

          term = g_delim;
        }

      /* Find the end of the string */

      for (pend = pbegin + 1;
           *pend && strchr(term, *pend) == NULL;
           pend++);

      /* pend either points to the end of the string or to
       * the first delimiter after the string.
       */

      if (*pend)
        {
          /* Turn the delimiter into a null terminator */

          *pend++ = '\0';
        }

      /* Save the pointer where we left off */

      *saveptr = pend;

#ifndef CONFIG_DISABLE_ENVIRON
      /* Check for references to environment variables */

      if (pbegin[0] == '$' && !quoted)
        {
          /* Check for built-in variables */

          if (strcmp(pbegin, g_exitstatus) == 0)
            {
              if (vtbl->np.np_fail)
                {
                  return (char*)g_failure;
                }
              else
                {
                  return (char*)g_success;
                }
            }

          /* Not a built-in? Return the value of the environment variable with this name */

          else
            {
              char *value = getenv(pbegin+1);
              if (value)
                {
                  return value;
                }
              else
                {
                  return (char*)"";
                }
            }
        }
#endif
    }

  /* Return the beginning of the token. */

  return pbegin;
}

/****************************************************************************
 * Name: nsh_cmdenabled
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLESCRIPT
static inline bool nsh_cmdenabled(FAR struct nsh_vtbl_s *vtbl)
{
  struct nsh_parser_s *np = &vtbl->np;
  bool ret = !np->np_st[np->np_ndx].ns_disabled;
  if (ret)
    {
      switch (np->np_st[np->np_ndx].ns_state)
        {
          case NSH_PARSER_NORMAL :
          case NSH_PARSER_IF:
          default:
            break;

          case NSH_PARSER_THEN:
            ret = !np->np_st[np->np_ndx].ns_ifcond;
            break;

          case NSH_PARSER_ELSE:
            ret = np->np_st[np->np_ndx].ns_ifcond;
            break;
        }
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: nsh_ifthenelse
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLESCRIPT
static inline int nsh_ifthenelse(FAR struct nsh_vtbl_s *vtbl, FAR char **ppcmd, FAR char **saveptr)
{
  struct nsh_parser_s *np = &vtbl->np;
  FAR char *cmd = *ppcmd;
  bool disabled;

  if (cmd)
    {
      /* Check if the command is preceeded by "if" */

      if (strcmp(cmd, "if") == 0)
        {
          /* Get the cmd following the if */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (!*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "if");
              goto errout;
            }

          /* Verify that "if" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_NORMAL &&
              np->np_st[np->np_ndx].ns_state != NSH_PARSER_THEN &&
              np->np_st[np->np_ndx].ns_state != NSH_PARSER_ELSE)
            {
              nsh_output(vtbl, g_fmtcontext, "if");
              goto errout;
            }

          /* Check if we have exceeded the maximum depth of nesting */

          if (np->np_ndx >= CONFIG_NSH_NESTDEPTH-1)
            {
              nsh_output(vtbl, g_fmtdeepnesting, "if");
              goto errout;
            }

          /* "Push" the old state and set the new state */

          disabled                          = !nsh_cmdenabled(vtbl);
          np->np_ndx++;
          np->np_st[np->np_ndx].ns_state    = NSH_PARSER_IF;
          np->np_st[np->np_ndx].ns_disabled = disabled;
          np->np_st[np->np_ndx].ns_ifcond   = false;
        }
      else if (strcmp(cmd, "then") == 0)
        {
          /* Get the cmd following the then -- there shouldn't be one */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "then");
              goto errout;
            }

          /* Verify that "then" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_IF)
            {
              nsh_output(vtbl, g_fmtcontext, "then");
              goto errout;
            }
          np->np_st[np->np_ndx].ns_state = NSH_PARSER_THEN;
        }
      else if (strcmp(cmd, "else") == 0)
        {
          /* Get the cmd following the else -- there shouldn't be one */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "else");
              goto errout;
            }

          /* Verify that "then" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_THEN)
            {
              nsh_output(vtbl, g_fmtcontext, "else");
              goto errout;
            }
          np->np_st[np->np_ndx].ns_state = NSH_PARSER_ELSE;
        }
      else if (strcmp(cmd, "fi") == 0)
        {
          /* Get the cmd following the fi -- there should be one */

          *ppcmd = nsh_argument(vtbl, saveptr);
          if (*ppcmd)
            {
              nsh_output(vtbl, g_fmtarginvalid, "fi");
              goto errout;
            }

          /* Verify that "fi" is valid in this context */

          if (np->np_st[np->np_ndx].ns_state != NSH_PARSER_THEN &&
              np->np_st[np->np_ndx].ns_state != NSH_PARSER_ELSE)
            {
              nsh_output(vtbl, g_fmtcontext, "fi");
              goto errout;
            }

          if (np->np_ndx < 1) /* Shouldn't happen */
            {
              nsh_output(vtbl, g_fmtinternalerror, "if");
              goto errout;
            }

          /* "Pop" the previous state */

          np->np_ndx--;
        }
      else if (np->np_st[np->np_ndx].ns_state == NSH_PARSER_IF)
        {
          nsh_output(vtbl, g_fmtcontext, cmd);
          goto errout;
        }
    }
  return OK;

errout:
  np->np_ndx               = 0;
  np->np_st[0].ns_state    = NSH_PARSER_NORMAL;
  np->np_st[0].ns_disabled = false;
  np->np_st[0].ns_ifcond   = false;
  return ERROR;
}
#endif

/****************************************************************************
 * Name: nsh_saveresult
 ****************************************************************************/

static inline int nsh_saveresult(FAR struct nsh_vtbl_s *vtbl, bool result)
{
  struct nsh_parser_s *np = &vtbl->np;

#ifndef CONFIG_NSH_DISABLESCRIPT
  if (np->np_st[np->np_ndx].ns_state == NSH_PARSER_IF)
    {
      np->np_fail = false;
      np->np_st[np->np_ndx].ns_ifcond = result;
      return OK;
    }
  else
#endif
    {
      np->np_fail = result;
      return result ? ERROR : OK;
    }
}

/****************************************************************************
 * Name: nsh_nice
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static inline int nsh_nice(FAR struct nsh_vtbl_s *vtbl, FAR char **ppcmd, FAR char **saveptr)
{
  FAR char *cmd = *ppcmd;

  vtbl->np.np_nice = 0;
  if (cmd)
    {
      /* Check if the command is preceded by "nice" */

      if (strcmp(cmd, "nice") == 0)
        {
          /* Nicenesses range from -20 (most favorable scheduling) to 19
           * (least  favorable).  Default is 10.
           */

          vtbl->np.np_nice = 10;

          /* Get the cmd (or -d option of nice command) */

          cmd = nsh_argument(vtbl, saveptr);
          if (cmd && strcmp(cmd, "-d") == 0)
            {
              FAR char *val = nsh_argument(vtbl, saveptr);
              if (val)
                {
                  char *endptr;
                  vtbl->np.np_nice = (int)strtol(val, &endptr, 0);
                  if (vtbl->np.np_nice > 19 || vtbl->np.np_nice < -20 ||
                      endptr == val || *endptr != '\0')
                    {
                      nsh_output(vtbl, g_fmtarginvalid, "nice");
                      return ERROR;
                    }
                  cmd = nsh_argument(vtbl, saveptr);
                }
            }

          /* Return the real command name */

          *ppcmd = cmd;
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_parse
 *
 * Description:
 *   This function parses and executes one NSH command.
 *
 ****************************************************************************/

int nsh_parse(FAR struct nsh_vtbl_s *vtbl, char *cmdline)
{
  FAR char *argv[MAX_ARGV_ENTRIES];
  FAR char *saveptr;
  FAR char *cmd;
  FAR char *redirfile = NULL;
  int       fd = -1;
  int       oflags = 0;
  int       argc;
  int       ret;

  /* Initialize parser state */

  memset(argv, 0, MAX_ARGV_ENTRIES*sizeof(FAR char *));
#ifndef CONFIG_NSH_DISABLEBG
  vtbl->np.np_bg       = false;
#endif
  vtbl->np.np_redirect = false;

  /* Parse out the command at the beginning of the line */

  saveptr = cmdline;
  cmd = nsh_argument(vtbl, &saveptr);

  /* Handler if-then-else-fi */

#ifndef CONFIG_NSH_DISABLESCRIPT
  if (nsh_ifthenelse(vtbl, &cmd, &saveptr) != 0)
    {
      goto errout;
    }
#endif

  /* Handle nice */

#ifndef CONFIG_NSH_DISABLEBG
  if (nsh_nice(vtbl, &cmd, &saveptr) != 0)
    {
      goto errout;
    }
#endif

  /* Check if any command was provided -OR- if command processing is
   * currently disabled.
   */

#ifndef CONFIG_NSH_DISABLESCRIPT
  if (!cmd || !nsh_cmdenabled(vtbl))
#else
  if (!cmd)
#endif
    {
      /* An empty line is not an error and an unprocessed command cannot
       * generate an error, but neither should they change the last
       * command status.
       */

      return OK;
    }

  /* Parse all of the arguments following the command name.  The form
   * of argv is:
   *
   *   argv[0]:      The command name.
   *   argv[1]:      The beginning of argument (up to CONFIG_NSH_MAXARGUMENTS)
   *   argv[argc-3]: Possibly '>' or '>>'
   *   argv[argc-2]: Possibly <file>
   *   argv[argc-1]: Possibly '&'
   *   argv[argc]:   NULL terminating pointer
   *
   * Maximum size is CONFIG_NSH_MAXARGUMENTS+5
   */

  argv[0] = cmd;
  for (argc = 1; argc < MAX_ARGV_ENTRIES-1; argc++)
    {
      argv[argc] = nsh_argument(vtbl, &saveptr);
      if (!argv[argc])
        {
          break;
        }
    }

  argv[argc] = NULL;

  /* Check if the command should run in background */

#ifndef CONFIG_NSH_DISABLEBG
  if (argc > 1 && strcmp(argv[argc-1], "&") == 0)
    {
      vtbl->np.np_bg = true;
      argv[argc-1] = NULL;
      argc--;
    }
#endif

  /* Check if the output was re-directed using > or >> */

  if (argc > 2)
    {
      /* Check for redirection to a new file */

      if (strcmp(argv[argc-2], g_redirect1) == 0)
        {
          vtbl->np.np_redirect = true;
          oflags               = O_WRONLY|O_CREAT|O_TRUNC;
          redirfile            = nsh_getfullpath(vtbl, argv[argc-1]);
          argc                -= 2;
        }

      /* Check for redirection by appending to an existing file */

      else if (strcmp(argv[argc-2], g_redirect2) == 0)
        {
          vtbl->np.np_redirect = true;
          oflags               = O_WRONLY|O_CREAT|O_APPEND;
          redirfile            = nsh_getfullpath(vtbl, argv[argc-1]);
          argc                -= 2;
        }
    }

  /* Check if the maximum number of arguments was exceeded */

  if (argc > CONFIG_NSH_MAXARGUMENTS)
    {
      nsh_output(vtbl, g_fmttoomanyargs, cmd);
    }

  /* Does this command correspond to an application filename?
   * nsh_fileapp() returns:
   *
   *   -1 (ERROR)  if the application task corresponding to 'argv[0]' could not
   *               be started (possibly because it doesn not exist).
   *    0 (OK)     if the application task corresponding to 'argv[0]' was
   *               and successfully started.  If CONFIG_SCHED_WAITPID is
   *               defined, this return value also indicates that the
   *               application returned successful status (EXIT_SUCCESS)
   *    1          If CONFIG_SCHED_WAITPID is defined, then this return value
   *               indicates that the application task was spawned successfully
   *               but returned failure exit status.
   *
   * Note the priority if not effected by nice-ness.
   */

#ifdef CONFIG_NSH_FILE_APPS
  ret = nsh_fileapp(vtbl, argv[0], argv, redirfile, oflags);
  if (ret >= 0)
    {
      /* nsh_fileapp() returned 0 or 1.  This means that the builtin
       * command was successfully started (although it may not have ran
       * successfully).  So certainly it is not an NSH command.
       */

      return nsh_saveresult(vtbl, ret != OK);
    }

  /* No, not a built in command (or, at least, we were unable to start a
   * builtin command of that name).  Treat it like an NSH command.
   */

#endif

  /* Does this command correspond to a builtin command?
   * nsh_builtin() returns:
   *
   *   -1 (ERROR)  if the application task corresponding to 'argv[0]' could not
   *               be started (possibly because it doesn not exist).
   *    0 (OK)     if the application task corresponding to 'argv[0]' was
   *               and successfully started.  If CONFIG_SCHED_WAITPID is
   *               defined, this return value also indicates that the
   *               application returned successful status (EXIT_SUCCESS)
   *    1          If CONFIG_SCHED_WAITPID is defined, then this return value
   *               indicates that the application task was spawned successfully
   *               but returned failure exit status.
   *
   * Note the priority if not effected by nice-ness.
   */

#if defined(CONFIG_NSH_BUILTIN_APPS) && (!defined(CONFIG_NSH_FILE_APPS) || !defined(CONFIG_FS_BINFS))
  ret = nsh_builtin(vtbl, argv[0], argv, redirfile, oflags);
  if (ret >= 0)
    {
      /* nsh_builtin() returned 0 or 1.  This means that the builtin
       * command was successfully started (although it may not have ran
       * successfully).  So certainly it is not an NSH command.
       */

      return nsh_saveresult(vtbl, ret != OK);
    }

  /* No, not a built in command (or, at least, we were unable to start a
   * builtin command of that name).  Treat it like an NSH command.
   */

#endif

  /* Redirected output? */

  if (vtbl->np.np_redirect)
    {
      /* Open the redirection file.  This file will eventually
       * be closed by a call to either nsh_release (if the command
       * is executed in the background) or by nsh_undirect if the
       * command is executed in the foreground.
       */

      fd = open(redirfile, oflags, 0666);
      nsh_freefullpath(redirfile);
      redirfile = NULL;

      if (fd < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "open", NSH_ERRNO);
          goto errout;
        }
    }

  /* Handle the case where the command is executed in background.
   * However is app is to be started as builtin new process will
   * be created anyway, so skip this step.
   */

#ifndef CONFIG_NSH_DISABLEBG
  if (vtbl->np.np_bg)
    {
      struct sched_param param;
      struct nsh_vtbl_s *bkgvtbl;
      struct cmdarg_s *args;
      pthread_attr_t attr;
      pthread_t thread;

      /* Get a cloned copy of the vtbl with reference count=1.
       * after the command has been processed, the nsh_release() call
       * at the end of nsh_child() will destroy the clone.
       */

      bkgvtbl = nsh_clone(vtbl);
      if (!bkgvtbl)
        {
          goto errout_with_redirect;
        }

      /* Create a container for the command arguments */

      args = nsh_cloneargs(bkgvtbl, fd, argc, argv);
      if (!args)
        {
          nsh_release(bkgvtbl);
          goto errout_with_redirect;
        }

      /* Handle redirection of output via a file descriptor */

      if (vtbl->np.np_redirect)
        {
          (void)nsh_redirect(bkgvtbl, fd, NULL);
        }

      /* Get the execution priority of this task */

      ret = sched_getparam(0, &param);
      if (ret != 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "sched_getparm", NSH_ERRNO);
          nsh_releaseargs(args);
          nsh_release(bkgvtbl);
          goto errout;
        }

      /* Determine the priority to execute the command */

      if (vtbl->np.np_nice != 0)
        {
          int priority = param.sched_priority - vtbl->np.np_nice;
          if (vtbl->np.np_nice < 0)
            {
              int max_priority = sched_get_priority_max(SCHED_NSH);
              if (priority > max_priority)
                {
                  priority = max_priority;
                }
            }
          else
            {
              int min_priority = sched_get_priority_min(SCHED_NSH);
              if (priority < min_priority)
                {
                  priority = min_priority;
                }
            }

          param.sched_priority = priority;
        }

      /* Set up the thread attributes */

      (void)pthread_attr_init(&attr);
      (void)pthread_attr_setschedpolicy(&attr, SCHED_NSH);
      (void)pthread_attr_setschedparam(&attr, &param);

      /* Execute the command as a separate thread at the appropriate priority */

      ret = pthread_create(&thread, &attr, nsh_child, (pthread_addr_t)args);
      if (ret != 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "pthread_create", NSH_ERRNO_OF(ret));
          nsh_releaseargs(args);
          nsh_release(bkgvtbl);
          goto errout;
        }

      nsh_output(vtbl, "%s [%d:%d]\n", cmd, thread, param.sched_priority);
    }
  else
#endif
    {
      uint8_t save[SAVE_SIZE];

      /* Handle redirection of output via a file descriptor */

      if (vtbl->np.np_redirect)
        {
          nsh_redirect(vtbl, fd, save);
        }

      /* Then execute the command in "foreground" -- i.e., while the user waits
       * for the next prompt.  nsh_execute will return:
       *
       * -1 (ERRROR) if the command was unsuccessful
       *  0 (OK)     if the command was successful
       */

      ret = nsh_execute(vtbl, argc, argv);

      /* Restore the original output.  Undirect will close the redirection
       * file descriptor.
       */

      if (vtbl->np.np_redirect)
        {
          nsh_undirect(vtbl, save);
        }

      /* Mark errors so that it is possible to test for non-zero return values
       * in nsh scripts.
       */

      if (ret < 0)
        {
          goto errout;
        }
    }

  /* Return success if the command succeeded (or at least, starting of the
   * command task succeeded).
   */

  return nsh_saveresult(vtbl, false);

#ifndef CONFIG_NSH_DISABLEBG
errout_with_redirect:
  if (vtbl->np.np_redirect)
    {
      close(fd);
    }
#endif
errout:
  return nsh_saveresult(vtbl, true);
}
