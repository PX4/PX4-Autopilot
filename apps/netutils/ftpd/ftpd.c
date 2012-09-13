/****************************************************************************
 * apps/n etutils/ftpd.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes original code as well as logic adapted from hwport_ftpd, written
 * by Jaehyuk Cho <minzkn@minzkn.com> which is released under a BSD license.
 *
 *   Copyright (C) hwport.com. All rights reserved.
 *   Author: Jaehyuk Cho <mailto:minzkn@minzkn.com>
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

#include <sys/socket.h>
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <poll.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>

#include <apps/netutils/ftpd.h>

#include "ftpd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __NUTTX__ 1 /* Flags some unusual NuttX dependencies */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Account functions */

static FAR struct ftpd_account_s *ftpd_account_new(FAR const char *user,
              uint8_t accountflags);
static void ftpd_account_free(FAR struct ftpd_account_s *account);
static int  ftpd_account_setpassword(FAR struct ftpd_account_s *account,
              FAR const char *passwd);
static int  ftpd_account_add(FAR struct ftpd_server_s *server,
              FAR struct ftpd_account_s *account);
static int  ftpd_account_sethome(FAR struct ftpd_account_s *account,
              FAR const char *home);
static FAR struct ftpd_account_s *
              ftpd_account_search_user(FAR struct ftpd_session_s *session,
              FAR const char *user);
static FAR struct ftpd_account_s *
              ftpd_account_login(FAR struct ftpd_session_s *session,
              FAR const char *user, FAR const char *passwd);

/* Parsing functions */

static FAR char *ftpd_strtok(bool skipspace, FAR const char *delimiters,
              FAR char **str);
static FAR char *ftpd_strtok_alloc(bool skipspace,
              FAR const char *delimiters, FAR const char **str);

/* Socket helpers */

static int  ftpd_rxpoll(int sd, int timeout);
static int  ftpd_txpoll(int sd, int timeout);
static int  ftpd_accept(int sd, FAR void *addr, FAR socklen_t *addrlen,
              int timeout);
static ssize_t ftpd_recv(int sd, FAR void *data, size_t size, int timeout);
static ssize_t ftpd_send(int sd, FAR const void *data, size_t size,
              int timeout);
static ssize_t ftpd_response(int sd, int timeout, FAR const char *fmt, ...);

static int  ftpd_dataopen(FAR struct ftpd_session_s *session);
static int  ftpd_dataclose(FAR struct ftpd_session_s *session);
static FAR struct ftpd_server_s *ftpd_openserver(int port);

/* Path helpers */

static int ftpd_pathignore(FAR struct ftpd_pathnode_s *currpath);
static void ftpd_nodefree(FAR struct ftpd_pathnode_s *node);
static FAR struct ftpd_pathnode_s *ftpd_path2node(FAR const char *path);
static FAR char *ftpd_node2path(FAR struct ftpd_pathnode_s *node,
              bool strip);
static FAR struct ftpd_pathnode_s *
              ftpd_nodeappend(FAR struct ftpd_pathnode_s *head,
              FAR struct ftpd_pathnode_s *node, bool override);
static int  ftpd_getpath(FAR struct ftpd_session_s *session,
              FAR const char *path, FAR char **abspath,
              FAR char **workpath);

/* Commmand helpers */

static int  ftpd_changedir(FAR struct ftpd_session_s *session,
              FAR const char *rempath);
static off_t ftpd_offsatoi(FAR const char *filename, off_t offset);
static int ftpd_stream(FAR struct ftpd_session_s *session, int cmdtype);
static uint8_t ftpd_listoption(FAR char **param);
static int  ftpd_listbuffer(FAR struct ftpd_session_s *session,
              FAR char *path, FAR struct stat *st, FAR char *buffer,
              size_t buflen, unsigned int opton);
static int  fptd_listscan(FAR struct ftpd_session_s *session,
              FAR char *path, unsigned int opton);
static int  ftpd_list(FAR struct ftpd_session_s *session,
              unsigned int opton);

/* Command handlers */

static int ftpd_command_user(FAR struct ftpd_session_s *session);
static int ftpd_command_pass(FAR struct ftpd_session_s *session);
static int ftpd_command_syst(FAR struct ftpd_session_s *session);
static int ftpd_command_type(FAR struct ftpd_session_s *session);
static int ftpd_command_mode(FAR struct ftpd_session_s *session);
static int ftpd_command_abor(FAR struct ftpd_session_s *session);
static int ftpd_command_quit(FAR struct ftpd_session_s *session);
static int ftpd_command_noop(FAR struct ftpd_session_s *session);
static int ftpd_command_port(FAR struct ftpd_session_s *session);
static int ftpd_command_eprt(FAR struct ftpd_session_s *session);
static int ftpd_command_pwd(FAR struct ftpd_session_s *session);
static int ftpd_command_cwd(FAR struct ftpd_session_s *session);
static int ftpd_command_cdup(FAR struct ftpd_session_s *session);
static int ftpd_command_rmd(FAR struct ftpd_session_s *session);
static int ftpd_command_mkd(FAR struct ftpd_session_s *session);
static int ftpd_command_dele(FAR struct ftpd_session_s *session);
static int ftpd_command_pasv(FAR struct ftpd_session_s *session);
static int ftpd_command_epsv(FAR struct ftpd_session_s *session);
static int ftpd_command_list(FAR struct ftpd_session_s *session);
static int ftpd_command_nlst(FAR struct ftpd_session_s *session);
static int ftpd_command_acct(FAR struct ftpd_session_s *session);
static int ftpd_command_size(FAR struct ftpd_session_s *session);
static int ftpd_command_stru(FAR struct ftpd_session_s *session);
static int ftpd_command_rnfr(FAR struct ftpd_session_s *session);
static int ftpd_command_rnto(FAR struct ftpd_session_s *session);
static int ftpd_command_retr(FAR struct ftpd_session_s *session);
static int ftpd_command_stor(FAR struct ftpd_session_s *session);
static int ftpd_command_appe(FAR struct ftpd_session_s *session);
static int ftpd_command_rest(FAR struct ftpd_session_s *session);
static int ftpd_command_mdtm(FAR struct ftpd_session_s *session);
static int ftpd_command_opts(FAR struct ftpd_session_s *session);
static int ftpd_command_site(FAR struct ftpd_session_s *session);
static int ftpd_command_help(FAR struct ftpd_session_s *session);

static int ftpd_command(FAR struct ftpd_session_s *session);

/* Worker thread */

static int  ftpd_startworker(pthread_startroutine_t handler, FAR void *arg,
              size_t stacksize);
static void ftpd_freesession(FAR struct ftpd_session_s *session);
static void ftpd_workersetup(FAR struct ftpd_session_s *session);
static FAR void *ftpd_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ftpd_cmd_s g_ftpdcmdtab[] =
{
  {"USER", ftpd_command_user, 0},                  /* USER <SP> <username> <CRLF> */
  {"PASS", ftpd_command_pass, 0},                  /* PASS <SP> <password> <CRLF> */
  {"SYST", ftpd_command_syst, FTPD_CMDFLAG_LOGIN}, /* SYST <CRLF> */
  {"TYPE", ftpd_command_type, FTPD_CMDFLAG_LOGIN}, /* TYPE <SP> <type-code> <CRLF> */
  {"MODE", ftpd_command_mode, FTPD_CMDFLAG_LOGIN}, /* MODE <SP> <mode-code> <CRLF> */
  {"ABOR", ftpd_command_abor, FTPD_CMDFLAG_LOGIN}, /* ABOR <CRLF> */
  {"QUIT", ftpd_command_quit, 0},                  /* QUIT <CRLF> */
  {"NOOP", ftpd_command_noop, FTPD_CMDFLAG_LOGIN}, /* NOOP <CRLF> */
  {"PORT", ftpd_command_port, FTPD_CMDFLAG_LOGIN}, /* PORT <SP> <host-port> <CRLF> */
  {"EPRT", ftpd_command_eprt, FTPD_CMDFLAG_LOGIN}, /* EPRT <SP> <d> <net-prt> <d> <net-addr> <d> <tcp-port> <d> <CRLF> */
  {"PWD" , ftpd_command_pwd , FTPD_CMDFLAG_LOGIN}, /* PWD  <CRLF> */
  {"XPWD", ftpd_command_pwd , FTPD_CMDFLAG_LOGIN}, /* XPWD <CRLF> */
  {"CWD" , ftpd_command_cwd , FTPD_CMDFLAG_LOGIN}, /* CWD  <SP> <pathname> <CRLF> */
  {"XCWD", ftpd_command_cwd , FTPD_CMDFLAG_LOGIN}, /* XCWD <SP> <pathname> <CRLF> */
  {"CDUP", ftpd_command_cdup, FTPD_CMDFLAG_LOGIN}, /* CDUP <CRLF> */
  {"XCUP", ftpd_command_cdup, FTPD_CMDFLAG_LOGIN}, /* XCUP <CRLF> */
  {"RMD" , ftpd_command_rmd , FTPD_CMDFLAG_LOGIN}, /* RMD  <SP> <pathname> <CRLF> */
  {"XRMD", ftpd_command_rmd , FTPD_CMDFLAG_LOGIN}, /* XRMD <SP> <pathname> <CRLF> */
  {"MKD" , ftpd_command_mkd , FTPD_CMDFLAG_LOGIN}, /* MKD  <SP> <pathname> <CRLF> */
  {"XMKD", ftpd_command_mkd , FTPD_CMDFLAG_LOGIN}, /* XMKD <SP> <pathname> <CRLF> */
  {"DELE", ftpd_command_dele, FTPD_CMDFLAG_LOGIN}, /* DELE <SP> <pathname> <CRLF> */
  {"PASV", ftpd_command_pasv, FTPD_CMDFLAG_LOGIN}, /* PASV <CRLF> */
  {"EPSV", ftpd_command_epsv, FTPD_CMDFLAG_LOGIN}, /* EPSV <SP> <net-prt> <CRLF> OR EPSV <SP> ALL <CRLF> */ 
  {"LPSV", ftpd_command_epsv, FTPD_CMDFLAG_LOGIN}, /* LPSV ??? */
  {"LIST", ftpd_command_list, FTPD_CMDFLAG_LOGIN}, /* LIST [<SP> <pathname>] <CRLF> */
  {"NLST", ftpd_command_nlst, FTPD_CMDFLAG_LOGIN}, /* NLST [<SP> <pathname>] <CRLF> */
  {"ACCT", ftpd_command_acct, FTPD_CMDFLAG_LOGIN}, /* ACCT <SP> <account-information> <CRLF> */
  {"SIZE", ftpd_command_size, FTPD_CMDFLAG_LOGIN}, /* SIZE <SP> <pathname> <CRLF> */
  {"STRU", ftpd_command_stru, FTPD_CMDFLAG_LOGIN}, /* STRU <SP> <structure-code> <CRLF> */
  {"RNFR", ftpd_command_rnfr, FTPD_CMDFLAG_LOGIN}, /* RNFR <SP> <pathname> <CRLF> */
  {"RNTO", ftpd_command_rnto, FTPD_CMDFLAG_LOGIN}, /* RNTO <SP> <pathname> <CRLF> */
  {"RETR", ftpd_command_retr, FTPD_CMDFLAG_LOGIN}, /* RETR <SP> <pathname> <CRLF> */
  {"STOR", ftpd_command_stor, FTPD_CMDFLAG_LOGIN}, /* STOR <SP> <pathname> <CRLF> */
  {"APPE", ftpd_command_appe, FTPD_CMDFLAG_LOGIN}, /* APPE <SP> <pathname> <CRLF> */
  {"REST", ftpd_command_rest, FTPD_CMDFLAG_LOGIN}, /* REST <SP> <marker> <CRLF> */
  {"MDTM", ftpd_command_mdtm, FTPD_CMDFLAG_LOGIN}, /* MDTM <SP> <pathname> <CRLF> */
  {"OPTS", ftpd_command_opts, FTPD_CMDFLAG_LOGIN}, /* OPTS <SP> <option> <value> <CRLF> */
  {"SITE", ftpd_command_site, FTPD_CMDFLAG_LOGIN}, /* SITE <SP> <string> <CRLF> */
  {"HELP", ftpd_command_help, FTPD_CMDFLAG_LOGIN}, /* HELP [<SP> <string>] <CRLF> */
#if 0
  {"SMNT", ftpd_command_smnt, FTPD_CMDFLAG_LOGIN}, /* SMNT <SP> <pathname> <CRLF> */
  {"REIN", ftpd_command_rein, FTPD_CMDFLAG_LOGIN}, /* REIN <CRLF> */
  {"STOU", ftpd_command_stou, FTPD_CMDFLAG_LOGIN}, /* STOU <CRLF> */
  {"STAT", ftpd_command_stat, FTPD_CMDFLAG_LOGIN}, /* STAT [<SP> <pathname>] <CRLF> */
  {"ALLO", ftpd_command_stat, FTPD_CMDFLAG_LOGIN}, /* ALLO <SP> <decimal-integer> [<SP> R <SP> <decimal-integer>] <CRLF> */
#endif
  {NULL, (ftpd_cmdhandler_t)0, 0} 
};

static const char g_cdup[]      = "..";
static const char g_respfmt1[]  = "%03u%c%s\r\n";   /* Integer, character, string */
static const char g_respfmt2[]  = "%03u%c%s%s\r\n"; /* Integer, character, two strings */

static const char *g_monthtab[] =
{
  "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

static const char *g_ftpdhelp[] =
{
  "The following commands are recognized (* =>'s unimplemented):",
  "CWD     XCWD    CDUP    XCUP    SMNT*   QUIT    PORT    PASV",
  "EPRT*   EPSV*   ALLO*   RNFR    RNTO    DELE    MDTM    RMD",
  "XRMD    MKD     XMKD    PWD     XPWD    SIZE    SYST    HELP",
  "NOOP    FEAT*   OPTS    AUTH*   CCC*    CONF*   ENC*    MIC*",
  "PBSZ*   PROT*   TYPE    STRU*   MODE*   RETR    STOR    STOU*",
  "APPE    REST    ABOR    USER    PASS    ACCT*   REIN*   LIST",
  "NLST    STAT*   SITE*   MLSD*   MLST*",
  "Direct comments to " CONFIG_FTPD_VENDORID,
   NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Account Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ftpd_account_new
 ****************************************************************************/

static FAR struct ftpd_account_s *ftpd_account_new(FAR const char *user,
                                                   uint8_t accountflags)
{
  FAR struct ftpd_account_s *ret;
  size_t usersize;
  size_t allocsize;

  /* Get the size of the allocation */

  allocsize = sizeof(struct ftpd_account_s);
  if (!user)
    {
      usersize = 0;
    }
  else
    {
      usersize = strlen(user);
      allocsize += usersize + 1;
    }

  /* Allocate the account and user string */

  ret = (struct ftpd_account_s *)zalloc(allocsize);
  if (!ret)
    {
      ndbg("Failed to allocate account\n");
      return NULL;
    }

  /* Initialize the account and user string */

  ret->flags = accountflags;

  if (user)
    {
      ret->user = (FAR char *)&ret[1];
      strcpy(ret->user, user);
    }

  return ret;
}

/****************************************************************************
 * Name: ftpd_account_free
 ****************************************************************************/

static void ftpd_account_free(FAR struct ftpd_account_s *account)
{
  struct ftpd_account_s *prev;
  DEBUGASSERT(account);

  /* Back up to the first entry in the list */

  while (account->blink)
    {
      account = account->blink;
    }

  /* Then free the entire list */

  while (account)
    {
      prev    = account;
      account = account->flink;

      /* Free the home path and the password */

      if (prev->home)
        {
          free(prev->home);
        }
    
      if (prev->password)
        {
          free(prev->password);
        }

      /* Then free the container itself */

      free(prev);
    }
}

/****************************************************************************
 * Name: ftpd_account_setpassword
 ****************************************************************************/

static int ftpd_account_setpassword(FAR struct ftpd_account_s *account,
                                   FAR const char *passwd)
{
  FAR char *temp;
  DEBUGASSERT(account);

  /* Make of copy of the password string (if it is non-null) */

  temp = NULL;
  if (passwd)
    {
      temp = strdup(passwd);
      if (!temp)
        {
          return -ENOMEM;
        }
    }

  /* Free any existing password string */
  
  if (account->password)
    {
      free(account->password);
    }

  /* Set the new password */

  account->password = temp;
  return OK;
}

/****************************************************************************
 * Name: ftpd_account_add
 ****************************************************************************/

static int ftpd_account_add(FAR struct ftpd_server_s *server,
                            FAR struct ftpd_account_s *account)
{
  FAR struct ftpd_account_s *head;
  FAR struct ftpd_account_s *tail;
  DEBUGASSERT(server && account);

  /* Find the beginning of the list */

  head = account;
  while (head->blink)
    {
      head = head->blink;
    }

  /* Find the tail of the list */

  tail = account;
  while (tail->flink)
    {
      tail = tail->flink;
    }

  /* Handle the case where the list is empty */
 
  if (!server->head)
    {
      server->head = head;
    }
  else
    {
      head->blink = server->tail;
      server->tail->flink = head;
    }

  server->tail = tail;
  return OK;
}

/****************************************************************************
 * Name: ftpd_account_sethome
 ****************************************************************************/

static int ftpd_account_sethome(FAR struct ftpd_account_s *account,
                                FAR const char *home)
{
  FAR char *temp;

  DEBUGASSERT(account);

  /* Make a copy of the home path string (unless it is NULL) */

  temp = NULL;
  if (home)
    {
      temp = strdup(home);
      if (!temp)
        {
          return -ENOMEM;
        }
    }

  /* Free any existing home path string */

  if (account->home)
    {
      free(account->home);
    }

  /* And set the new home path string */

  account->home = temp;
  return OK;
}

/****************************************************************************
 * Name: ftpd_account_search_user
 ****************************************************************************/

static FAR struct ftpd_account_s *
ftpd_account_search_user(FAR struct ftpd_session_s *session,
                         FAR const char *user)
{
  FAR struct ftpd_account_s *newaccount = NULL;
  FAR struct ftpd_account_s *account;
  uint8_t accountflags; 

  account = session->head;
  while (account)
    {
      accountflags = account->flags;

      /* Check if the account has a user */

      if (!account->user)
        {
          /* No.. The account has no user, was a user name provided? */

          if (!user)
            {
              /* Yes.. create the account */

              newaccount = ftpd_account_new(NULL, accountflags);
              if (newaccount)
                {
                  if (ftpd_account_setpassword(newaccount, account->password) < 0)
                    {
                      ftpd_account_free(newaccount);
                      newaccount = NULL;
                    }
                  else if (ftpd_account_sethome(newaccount, account->home) < 0)
                    {
                      ftpd_account_free(newaccount);
                      newaccount = NULL;
                    }
                }
              break;
            }
        }

      /* Was a user name provided? */

      else if (user)
        {
          /* Check if matches the user name on the account */

          if (strcmp(user, (FAR const char *)account->user) == 0)
            {
              /* Yes.. create the account */

              newaccount = ftpd_account_new(account->user, accountflags);
              if (newaccount)
                {
                  if (ftpd_account_setpassword(newaccount, account->password) != 0)
                    {
                      ftpd_account_free(newaccount);
                      newaccount = NULL;
                    }
                  else if (ftpd_account_sethome(newaccount, account->home) != 0)
                    {
                      ftpd_account_free(newaccount);
                      newaccount = NULL;
                    }
                }
              break;
            }
        }

      /* Try the next account */

      account = account->flink;
    }

  return newaccount;
}

/****************************************************************************
 * Name: ftpd_account_login
 ****************************************************************************/

static FAR struct ftpd_account_s *
ftpd_account_login(FAR struct ftpd_session_s *session,
                   FAR const char *user, FAR const char *passwd)
{
  FAR struct ftpd_account_s *account;
  bool pwvalid;
  FAR char *home;

  account = ftpd_account_search_user(session, user);
  if (!account)
    {
      return NULL;
    }

  if (!account->password)
    {
      if (!passwd)
        {
          pwvalid = true;
        }
      else if (passwd[0] == '\0')
        {
          pwvalid = true;
        }
      else
        {
          pwvalid = false;
        }
    }
  else if (!passwd)
    {
      pwvalid = false;
    }
  else if (strcmp(passwd, (FAR const char *)account->password) == 0)
    {
      pwvalid = true;
    }
  else
    {
      pwvalid = false;
    }

  if (!pwvalid)
    {
      ftpd_account_free(account);
      return NULL;
    }

  home = account->home;
  if (!home)
    {
      home = getenv("HOME");
    }

  if ((account->flags & FTPD_ACCOUNTFLAG_ADMIN) != 0)
    {
      /* admin user */

      session->home = strdup("/");
      session->work = strdup(!home ? "/" : home);
    }
  else
      {
        /* normal user */

        session->home = strdup(!home ? "/" : home);
        session->work = strdup("/");
      }

  ftpd_account_free(account);
  return account;
}

/****************************************************************************
 * Parsing Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ftpd_strtok
 ****************************************************************************/

static FAR char *ftpd_strtok(bool skipspace, FAR const char *delimiters,
                             FAR char **str)
{
  FAR const char *dptr;
  FAR char *sptr;
  FAR char *ret;
 
  sptr = *str;

  /* Skip any leading spaces */

  if (skipspace)
    {
      while (isspace(*sptr))
        {
          sptr++;
        }
    }

  ret = sptr;

  /* The following is an implementation of strtok.  It does not modify the
   * original string as strtok does, however.
   */

  while (*sptr != '\0')
    {
      dptr = delimiters;
      while (*sptr != *dptr && *dptr != '\0')
        {
          dptr++;
        }

      if (*sptr == *dptr)
        {
          break;
        }

      sptr++;
    }
    
    /* Save the place where we will resuming searching */

    *str = sptr;
    return ret;
}

/****************************************************************************
 * Name: ftpd_strtok_alloc
 ****************************************************************************/

static FAR char *ftpd_strtok_alloc(bool skipspace, FAR const char *delimiters,
                                   FAR const char **str)
{
  FAR const char *sptr;
  FAR const char *left;
  FAR const char *right;
  FAR const char *dptr;
  FAR char *ret;
  size_t tokenlen;

  sptr = *str;

  if (skipspace)
    {
      while (isspace(*sptr))
        {
          sptr++;
        }
    }

  right = sptr;
  left  = sptr;

  /* The the following logic is similar to strtok(), but only bounds the
   * token of interest between left (the first character of the substring)
   * and right (the character after the end of the substring).
   */

  while (*sptr != '\0')
    {
      dptr = delimiters;
      while (*sptr != *dptr && *dptr != '\0')
        {
          dptr++;
        }

      if (*sptr == *dptr)
        {
          break;
        }

      sptr++;

      /* Adjust the right pointer but ignoring any trailing spaces if
       * 'skipspace' is selected.
       */

      if (!skipspace || !isspace(*sptr))
        {
          right = sptr;
        }
    }

    /* Allocate memory large enough to hold the entire sub-string (including
     * the NUL terminator.
     */

    tokenlen = (size_t)(right - left);
    ret = (FAR char *)malloc(tokenlen + 1);
    if (ret)
      {
        if (tokenlen > 0)
          {
            memcpy(ret, left, tokenlen);
          }
        ret[tokenlen] = '\0';
      }

    /* Save the place where we will resuming searching */

    *str = sptr;
    return ret;
}

/****************************************************************************
 * Socket Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: ftpd_rxpoll
 ****************************************************************************/

static int ftpd_rxpoll(int sd, int timeout)
{
  struct pollfd fds[1];
  int ret;

  /* Set up the poll */

  fds[0].fd      = sd;
  fds[0].events  = POLLIN;
  fds[0].revents = 0;

  /* Perform the poll. */

  ret = poll(fds, 1, timeout);

  /* Handle the result of the poll.  On success, poll returns the number
   * of structures that have nonzero revents fields.  A value of 0 indicates
   * that the call timed out and no file descriptors were ready.  On error,
   * -1 is returned, and errno is set appropriately:
   */

  if (ret == 0)
    {
      //nvdbg("poll() timed out\n");
      return -ETIMEDOUT;
    }
  else if (ret < 0)
    {
      int errval = errno;
      nvdbg("poll() failed: %d\n", errval);
      return -errval;
    }
  else
    {
      return OK;
    }
}

/****************************************************************************
 * Name: ftpd_txpoll
 ****************************************************************************/

static int ftpd_txpoll(int sd, int timeout)
{
  struct pollfd fds[1];
  int ret;

  /* Set up the poll */

  fds[0].fd      = sd;
  fds[0].events  = POLLOUT;
  fds[0].revents = 0;

  /* Perform the poll. */

  ret = poll(fds, 1, timeout);

  /* Handle the result of the poll.  On success, poll returns the number
   * of structures that have nonzero revents fields.  A value of 0 indicates
   * that the call timed out and no file descriptors were ready.  On error,
   * -1 is returned, and errno is set appropriately:
   */

  if (ret == 0)
    {
      nvdbg("poll() timed out\n");
      return -ETIMEDOUT;
    }
  else if (ret < 0)
    {
      int errval = errno;
      nvdbg("poll() failed: %d\n", errval);
      return -errval;
    }
  else
    {
      return OK;
    }
}

/****************************************************************************
 * Name: ftpd_accept
 ****************************************************************************/

static int ftpd_accept(int sd, FAR void *addr, FAR socklen_t *addrlen,
                       int timeout)
{
  int acceptsd;
  int ret;

  /* Handle any requested timeout */

  if (timeout >= 0)
    {
      ret = ftpd_rxpoll(sd, timeout);
      if (ret < 0)
        {
          /* Only report interesting, infrequent errors (not the common timeout) */

#ifdef CONFIG_DEBUG_NET
          if (ret != -ETIMEDOUT)
            {
              ndbg("ftpd_rxpoll() failed: %d\n", ret);
            }
#endif
          return ret;
        }
    }

  /* Accept the connection -- waiting if necessary */

  acceptsd = accept(sd, (FAR struct sockaddr *)addr, addrlen);
  if (acceptsd < 0)
    {
      int errval = errno;
      ndbg("accept() failed: %d\n", errval);
      return -errval;
    }

  return acceptsd;
}

/****************************************************************************
 * Name: ftpd_recv
 ****************************************************************************/

static ssize_t ftpd_recv(int sd, FAR void *data, size_t size, int timeout)
{
  ssize_t ret;
  int status;

  /* Handle any requested timetout */

  if (timeout >= 0)
    {
      status = ftpd_rxpoll(sd, timeout);
      if (status < 0)
        {
          nvdbg("ftpd_rxpoll: %d\n", status);
          return (ssize_t)status;
        }
    }

  /* Receive the data... waiting if necessary.  The client side will break the
   * connection after the file has been sent.  Zero (end-of-file) should be
   * received in this case.
   */

  ret = recv(sd, data, size, 0);
  if (ret < 0)
    {
      int errval = errno;

      ndbg("recv() failed: %d\n", errval);
      return -errval;
    }

  return ret;
}

/****************************************************************************
 * Name: ftpd_send
 ****************************************************************************/

static ssize_t ftpd_send(int sd, FAR const void *data, size_t size, int timeout)
{
  ssize_t ret;
    
  /* Handle any requested timetout */

  if (timeout >= 0)
    {
      int status = ftpd_txpoll(sd, timeout);
      if (status < 0)
        {
          nvdbg("ftpd_rxpoll: %d\n", status);
          return (ssize_t)status;
        }
    }

  /* Then send the data (waiting if necessary) */

  ret = send(sd, data, size, 0);
  if (ret < 0)
    {
      ssize_t errval = errno;
      ndbg("send() failed: %d\n", errval);
      return -errval;
    }

  return ret;
}

/****************************************************************************
 * Name: ftpd_response
 ****************************************************************************/

static ssize_t ftpd_response(int sd, int timeout, FAR const char *fmt, ...)
{
  FAR char *buffer;
  ssize_t bytessent;
  va_list ap;

  va_start(ap, fmt);
  avsprintf(&buffer, fmt, ap);
  va_end(ap);

  if (!buffer)
    {
      return -ENOMEM;
    }

  bytessent = ftpd_send(sd, buffer, strlen(buffer), timeout);
  free(buffer);

  return bytessent;
}

/****************************************************************************
 * Name: ftpd_dataopen
 ****************************************************************************/

static int ftpd_dataopen(FAR struct ftpd_session_s *session)
{
  int sd;
  int ret;

  if (session->data.sd < 0)
    {
      /* PORT session */

#ifdef CONFIG_NET_IPv6
      if (session->data.addr.ss.ss_family == AF_INET6)
        {
          session->data.sd = socket(PF_INET6, SOCK_STREAM, IPPROTO_TCP);
        }
      else
        {
          session->data.sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        }
#else
      session->data.sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
#endif

      if (session->data.sd < 0)
        {
          int errval = errno;
          ndbg("socket() failed: %d\n", errval);
          (void)ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 451, ' ', "Socket error !");
          return -errval;
        }

      session->data.addrlen = (socklen_t)sizeof(session->data.addr);
      ret = connect(session->data.sd, (FAR const struct sockaddr *)(&session->data.addr),
                    session->data.addrlen);
      if (ret < 0)
        {
          int errval = errno;
          ndbg("connect() failed: %d\n", errval);
          (void)ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 451, ' ', "Connect error !");
          (void)ftpd_dataclose(session);
          return -errval;
        }

#ifdef CONFIG_NET_HAVE_SOLINGER
        {
          struct linger ling;

          (void)memset(&ling, 0, sizeof(ling));
          ling.l_onoff = 1;
          ling.l_linger = 4;
          (void)setsockopt(session->data.sd, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling));
        }
#endif

      return OK;
    }
    
  /* PASV session */

  session->data.addrlen = sizeof(session->data.addr);
  sd = ftpd_accept(session->data.sd, (struct sockaddr *)(&session->data.addr),
                  &session->data.addrlen, -1);
  if (sd < 0)
    {
      ndbg("ftpd_accept() failed: %d\n", sd);
      (void)ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 451, ' ', "Accept error !");
      (void)ftpd_dataclose(session);
      return sd;
    }

  close(session->data.sd);
  session->data.sd = sd;

#ifdef CONFIG_NET_HAVE_SOLINGER
  {
    struct linger ling;

    (void)memset(&ling, 0, sizeof(ling));
    ling.l_onoff = 1;
    ling.l_linger = 4;
    (void)setsockopt(session->data.sd, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling));
  }
#endif

  return OK;
}

/****************************************************************************
 * Name: ftpd_dataclose
 ****************************************************************************/

static int ftpd_dataclose(FAR struct ftpd_session_s *session)
{
  if (session->data.sd >= 0)
    {
      close(session->data.sd);
      session->data.sd = -1;
    }

  return OK;
}

/****************************************************************************
 * Name: ftpd_openserver
 ****************************************************************************/

static FAR struct ftpd_server_s *ftpd_openserver(int port)
{
  FAR struct ftpd_server_s *server;
  sa_family_t family;
  socklen_t addrlen;
  FAR const void *addr;
#if defined(SOMAXCONN)    
  int backlog = SOMAXCONN;
#else
  int backlog = 5;
#endif
  int ret;

  /* Allocate the server instance */

  server = (FAR struct ftpd_server_s *)zalloc(sizeof(struct ftpd_server_s));
  if (!server)
    {
      ndbg("Failed to allocate server\n");
      return NULL;
    }

  /* Initialize the server instance */

    server->sd   = -1;
    server->head = NULL;
    server->tail = NULL;

  /* Create the server listen socket */

#ifdef CONFIG_NET_IPv6
  server->sd = socket(PF_INET6, SOCK_STREAM, IPPROTO_TCP);
  if (server->sd < 0)
    {
      ftpd_close((FTPD_SESSION)server);
      return NULL;
    }

  family  = AF_INET6;
  addrlen = (socklen_t)sizeof(server->addr.in6); 
  addr    = (FAR void *)(&server->addr.in6);

  server->addr.in6.sin6_family   = family;
  server->addr.in6.sin6_flowinfo = 0;
  server->addr.in6.sin6_addr     = in6addr_any;
  server->addr.in6.sin6_port     = htons(port);
#else    
  server->sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (server->sd < 0)
    {
      ftpd_close((FTPD_SESSION)server);
      return NULL;
    }

  family  = AF_INET;
  addrlen = (socklen_t)sizeof(server->addr.in4); 
  addr    = (FAR void *)(&server->addr.in4);

  server->addr.in4.sin_family      = family;
  server->addr.in4.sin_addr.s_addr = htonl(INADDR_ANY);
  server->addr.in4.sin_port        = htons(port);
#endif    

#ifdef CONFIG_NET_HAVE_REUSEADDR
  {
    int reuse = 1;
   (void)setsockopt(server->sd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  }
#endif

  /* Bind the socket to the address */

  ret = bind(server->sd, (FAR const struct sockaddr *)addr, addrlen);
  if (ret < 0)
    {
      ftpd_close((FTPD_SESSION)server);
      return NULL;
    }

  /* Listen on the socket */

  ret = listen(server->sd, backlog);
  if (ret < 0)
    {
      ftpd_close((FTPD_SESSION)server);
      return NULL;
    }

  return (FTPD_SESSION)server;
}

/****************************************************************************
 * Path Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: ftpd_pathignore
 ****************************************************************************/

static int ftpd_pathignore(FAR struct ftpd_pathnode_s *currpath)
{
  FAR struct ftpd_pathnode_s *node;
  size_t namelen;

  namelen = !currpath->name ? 0 : strlen(currpath->name);

  if (namelen == 0)
    {
      if (currpath->blink)
        {
          currpath->ignore = true;
        }
        
      return OK;
    }

  if (strcmp(currpath->name, "..") == 0)
    {
      currpath->ignore = true;
        
      node = currpath->blink;
      while (node)
        {
          if (!node->ignore)
            {
              namelen = !node->name ? 0 : strlen(node->name);

              if (namelen > 0)
                {
                  node->ignore = true;
                }
              break;
            }
          node = node->blink;
        }

      return OK;
    }

  if (strcmp(currpath->name, ".") == 0)
    {
      currpath->ignore = true;
      return OK;
    }

  return OK;
}

/****************************************************************************
 * Name: 
 ****************************************************************************/

static void ftpd_nodefree(FAR struct ftpd_pathnode_s *node)
{
  FAR struct ftpd_pathnode_s *prev;

  while (node)
    {
      prev = node;
      node = node->flink;

      if (prev->name)
        {
          free(prev->name);
        }
      free(prev);
    }
}

/****************************************************************************
 * Name: ftpd_path2node
 ****************************************************************************/

static FAR struct ftpd_pathnode_s *ftpd_path2node(FAR const char *path)
{
  FAR struct ftpd_pathnode_s *head  = NULL;
  FAR struct ftpd_pathnode_s *tail = NULL;
  FAR struct ftpd_pathnode_s *newnode;
  FAR char *name;

  if (!path)
    {
      return NULL;
    }
     
  while (path[0] != '\0')
    {
      name = ftpd_strtok_alloc(false, "/\\", &path);
      if (!name)
        {
          break;
        }

      if (path[0] != '\0')
        {
          path++;
        }

      newnode = (FAR struct ftpd_pathnode_s *)malloc(sizeof(struct ftpd_pathnode_s));
      if (!newnode)
        {
          free(name);
          ftpd_nodefree(head);
          return NULL;
        }

      newnode->blink  = tail;
      newnode->flink  = NULL;
      newnode->ignore = false;
      newnode->name   = name;

      if (!tail)
        {
          head = newnode;
        }
      else
        {
          tail->flink = newnode;
        }

      tail = newnode;

      (void)ftpd_pathignore(newnode);
    }
     
  return head;
}

/****************************************************************************
 * Name: ftpd_node2path
 ****************************************************************************/

static FAR char *ftpd_node2path(FAR struct ftpd_pathnode_s *node,
                                   bool strip)
{
  FAR struct ftpd_pathnode_s *node1;
  FAR struct ftpd_pathnode_s *node2;
  FAR char *path;
  FAR size_t allocsize;
  FAR size_t namelen;

  if (!node)
    {
      return NULL;
    }

  allocsize = 0;
  node1 = node;
  while (node1)
    {
      if (strip)
        {
          if (node1->ignore)
            {
              node1 = node1->flink;
              continue;
            }
        }

      node2 = node1->flink;
      while (strip && node2)
        {
          if (!node2->ignore)
            {
                break;
            }

          node2 = node2->flink;
        }
        
      namelen = !node1->name ? 0 : strlen(node1->name);
      if (!node2)
        {
          if (namelen <= 0)
            {
              allocsize += 2;
            }
          else
            {
              allocsize += namelen +1;
            }
        }
      else
        {
          allocsize += namelen + 1;
        }

      node1 = node1->flink;
    }
   
  path = (FAR char *)malloc(allocsize);
  if (!path)
    {
      return NULL;
    }

  allocsize = 0;
  node1 = node;
  while (node1)
    {
      if (strip != 0)
        {
          if (node1->ignore)
            {
              node1 = node1->flink;
              continue;
            }
        }

      node2 = node1->flink;
      while (strip && node2)
        {
          if (!node2->ignore)
            {
              break;
            }

          node2 = node2->flink;
        }
        
      namelen = !node1->name ? 0 : strlen(node1->name);

      if (!node2)
        {
          if (namelen <= 0)
            {
              allocsize += sprintf(&path[allocsize], "/");
            }
          else
            {
              allocsize += sprintf(&path[allocsize], "%s", node1->name);
            }
        }
      else
        {
          allocsize += sprintf(&path[allocsize], "%s%s", node1->name, "/");
        }

      node1 = node1->flink;
    }

  return path;
}

/****************************************************************************
 * Name: ftpd_nodeappend
 ****************************************************************************/

static FAR struct ftpd_pathnode_s *
ftpd_nodeappend(FAR struct ftpd_pathnode_s *head,
                FAR struct ftpd_pathnode_s *node, bool override)
{
  FAR struct ftpd_pathnode_s *temp;

  if (override)
    {
      if (node && node->name && strlen(node->name) <= 0)
        {
          ftpd_nodefree(head);
          head = NULL;                    
        }
    }

  if (!head)
    {
      if (node)
        {
          node->blink = NULL;
        }

      head = node;
      node = NULL;
    }
        
  if (node)
    {
      temp = head;
      while (temp->flink)
        {
          temp = temp->flink;
        }

      node->blink = temp;
      temp->flink = node;
    }
        
  /* clear ignore */

  temp = head;
  while (temp)
    {
      temp->ignore = false;
      temp = temp->flink;
    }

  /* restrip */

  temp = head;
  while (temp)
    {
      (void)ftpd_pathignore(temp);
      temp = temp->flink;
    }

  return head;
}

/****************************************************************************
 * Name: ftpd_getpath
 ****************************************************************************/

static int ftpd_getpath(FAR struct ftpd_session_s *session,
                        FAR const char *path, FAR char **abspath,
                        FAR char **workpath)
{
  FAR struct ftpd_pathnode_s *abspath_node;
  FAR struct ftpd_pathnode_s *worknode;
  FAR struct ftpd_pathnode_s *appendnode;
  FAR char *abspath_local;
  FAR char *workpath_local;

  if (abspath)
    {
      *abspath = NULL;
    }

  if (workpath)
    {
      *workpath = NULL;
    }
    
  worknode = ftpd_path2node(!session->work ? "" : session->work);
  if (!worknode)
    {
      return -ENOMEM;
    }

  appendnode     = ftpd_path2node(path);
  worknode       = ftpd_nodeappend(worknode, appendnode, true);
  workpath_local = ftpd_node2path(worknode, 1);

  if (!workpath_local)
    {
      ftpd_nodefree(worknode);
      return -ENOMEM;
    }

  abspath_node = ftpd_path2node(!session->home ? "" : session->home);
  if (!abspath_node)
    {
      free(workpath_local);
      ftpd_nodefree(worknode);
      return -ENOMEM;
    }

  appendnode    = ftpd_path2node(workpath_local);
  abspath_node  = ftpd_nodeappend(abspath_node, appendnode, false);
  abspath_local = ftpd_node2path(abspath_node, 1);

  if (!abspath_local)
    {
      free(workpath_local);
      ftpd_nodefree(abspath_node);
      ftpd_nodefree(worknode);
      return -ENOMEM;
    }

  if (!workpath)
    {
      free(workpath_local);
    }
  else
    {
      *workpath = workpath_local;
    }

  if (!abspath)
    {
      free(abspath_local);
    }
  else
    {
      *abspath = abspath_local;
    }

  ftpd_nodefree(abspath_node);
  ftpd_nodefree(worknode);
  return OK;
}

/****************************************************************************
 * Command Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: ftpd_changedir
 ****************************************************************************/

static int ftpd_changedir(FAR struct ftpd_session_s *session,
                          FAR const char *rempath)
{
  FAR char *abspath;
  FAR char *workpath;
  struct stat st;
  int ret;

  ret = ftpd_getpath(session, rempath, (char **)(&abspath), (char **)(&workpath));
  if (ret < 0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not change directory !");
    }

  ret = stat(abspath, &st);
  if (ret < 0)
    {
      free(workpath);
      free(abspath);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt2, 550, ' ', rempath,
                           ": No such file or directory");
    }

  if (S_ISDIR(st.st_mode) == 0)
    {
      free(workpath);
      free(abspath);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt2, 550, ' ', rempath,
                           ": No such file or directory");
    }

  free(abspath);
  if (session->work)
    {
      free(session->work);
    }
  session->work = workpath;

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 250, ' ', "CWD command successful");
}

/****************************************************************************
 * Name: ftpd_offsatoi
 ****************************************************************************/

static off_t ftpd_offsatoi(FAR const char *filename, off_t offset)
{
  off_t ret;
  off_t temp;
  FILE *outstream;
  int ch;

  outstream = fopen(filename, "r");
  if (!outstream)
    {
      int errval = errno;
      ndbg("Failed to open %s: %d\n", filename, errval);
      return -errval;
    }

  ret  = 0; 
  temp = 0; 

  if (offset == (off_t)(-1))
    {
     for (;;)
        {
          ch = getc(outstream);
          if (ch == EOF)
            {
              break;
            }
          ret++;
          if (ch == '\n')
            {
              ret++;
            }
        }
        /* ret is ascii mode size */
    }
  else
    {
      while (offset < temp)
        {
          ch = getc(outstream);
          if (ch == EOF)
            {
              ret = -errno;
              break;
            }

          ret++;
          temp++;

          if (ch == '\n')
            {
              temp++;
            }
        }

      /* ret is binary mode offset */
    }

  (void)fclose(outstream);
  return ret;
}

/****************************************************************************
 * Name: ftpd_stream
 ****************************************************************************/

static int ftpd_stream(FAR struct ftpd_session_s *session, int cmdtype)
{
  FAR char *abspath;
  FAR char *path;
  bool isnew;
  int oflags;
  FAR char *buffer;
  size_t buflen;
  size_t wantsize;
  ssize_t rdbytes;
  ssize_t wrbytes;
  off_t pos = 0;
  int errval = 0;
  int ret;

  ret = ftpd_getpath(session, session->param, &abspath, NULL);
  if (ret < 0)
    {
      ftpd_response(session->cmd.sd, session->txtimeout,
                    g_respfmt1, 550, ' ', "Stream error !");
      goto errout;
    }
  path = abspath;

  ret = ftpd_dataopen(session);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  switch (cmdtype)
    {
      case 0: /* retr */
        oflags = O_RDONLY;
        break;

      case 1: /* stor */
        oflags = O_CREAT | O_WRONLY;
         break;

      case 2: /* appe */
        oflags = O_CREAT | O_WRONLY | O_APPEND;
        break;

      default:
        oflags = O_RDONLY;
        break;
    }

#if defined(O_LARGEFILE)
  oflags |= O_LARGEFILE;
#endif
#if defined(O_BINARY)
  oflags |= O_BINARY;
#endif

  /* Are we creating the file? */

  if ((oflags & O_CREAT) != 0)
    {
      int mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH;

      if (session->restartpos <= 0)
        {
          oflags |= O_TRUNC;
        }

      isnew = true;
      session->fd = open(path, oflags | O_EXCL, mode);
      if (session->fd < 0)
        {
          isnew = false;
          session->fd = open(path, oflags, mode);
        }
    }
  else
    {
      /* No.. we are opening an existing file */

      isnew = false;
      session->fd = open(path, oflags);
    }

  if (session->fd < 0)
    {
      ret = -errno;
      (void)ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 550, ' ', "Can not open file !");
      goto errout_with_data;
    }
    
  /* Restart position */

  if (session->restartpos > 0)
    {
      off_t seekoffs = (off_t)-1;
      off_t seekpos;

      /* Get the seek position */

      if (session->type == FTPD_SESSIONTYPE_A)
        {
          seekpos = ftpd_offsatoi(path, session->restartpos);
          if (seekpos < 0)
            {
              ndbg("ftpd_offsatoi failed: %d\n", seekpos);
              errval = -seekpos;
            }
        }
      else
        {
          seekpos = session->restartpos;
          if (seekpos < 0)
            {
              ndbg("Bad restartpos: %d\n", seekpos);
              errval = EINVAL;
            }
        }

      /* Seek to the request position */

      if (seekpos >= 0)
        {
          seekoffs = lseek(session->fd, seekpos, SEEK_SET);
          if (seekoffs < 0)
            {
              errval = errno;
              ndbg("lseek failed: %d\n", errval);
            }
        }

      /* Report errors.  If an error occurred, seekoffs will be negative and
       * errval will hold the (positive) error code.
       */

      if (seekoffs < 0)
        {
          (void)ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 550, ' ', "Can not seek file !");
          ret = -errval;
          goto errout_with_session;
        }

        pos += (off_t)seekoffs;
    }

  /* Send success message */

  ret = ftpd_response(session->cmd.sd, session->txtimeout,
                      g_respfmt1, 150, ' ', "Opening data connection");
  if (ret < 0)
    {
      ndbg("ftpd_response failed: %d\n", ret);
      goto errout_with_session;
    }

 for (;;)
    {
      /* Read from the source (file or TCP connection) */

      if (session->type == FTPD_SESSIONTYPE_A)
        {
          buffer   = &session->data.buffer[session->data.buflen >> 2];
          wantsize = session->data.buflen >> 2;
        }
      else
        {
          buffer   = session->data.buffer;
          wantsize = session->data.buflen;
        }

      if (cmdtype == 0)
        {
          /* Read from the file.  Read returns the error condition via errno. */

          rdbytes = read(session->fd, session->data.buffer, wantsize);
          if (rdbytes < 0)
            {
              errval = errno;
            }
        }
      else
        {
          /* Read from the TCP connection, ftpd_recve returns the negated error
           * condition.
           */

          rdbytes = ftpd_recv(session->data.sd, session->data.buffer,
                              wantsize, session->rxtimeout);
          if (rdbytes < 0)
            {
              errval = -rdbytes;
            }
        }

      /* A negative vaule of rdbytes indicates a read error.  errval has the
       * (positive) error code associated with the failure.
       */

      if (rdbytes < 0)
        {
          ndbg("Read failed: rdbytes=%d errval=%d\n", rdbytes, errval);
          (void)ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 550, ' ', "Data read error !");
          ret = -errval;
          break;
        }

      /* A value of rdbytes == 0 means that we have read the entire source
       * stream.
       */

      if (rdbytes == 0)
        {
          /* End-of-file */

          (void)ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 226, ' ', "Transfer complete");

          /* Return success */

          ret = 0;
          break;
        }

      /* Write to the destination (file or TCP connection) */

      if (session->type == FTPD_SESSIONTYPE_A)
        {
          /* Change to ascii */

          size_t offset = 0;
          buflen = 0;
          while (offset < ((size_t)rdbytes))
            {
              if (session->data.buffer[offset] == '\n')
                {
                  buffer[buflen++] = '\r';
                }
              buffer[buflen++] = session->data.buffer[offset++];
            }
        }
      else
        {
          buffer = session->data.buffer;
          buflen = (size_t)rdbytes;
        }

      if (cmdtype == 0)
        {
          /* Write to the TCP connection */

          wrbytes = ftpd_send(session->data.sd, buffer, buflen, session->txtimeout);
          if (wrbytes < 0)
            {
              errval = -wrbytes;
              ndbg("ftpd_send failed: %d\n", errval);
            }
        }
      else
        {
          /* Write to the file */

          wrbytes = write(session->fd, buffer, buflen);
          if (wrbytes < 0)
            {
              errval = errno;
              ndbg("write() failed: %d\n", errval);
            }
        }

      /* If the number of bytes returned by the write is not equal to the
       * number that we wanted to write, then an error (or at least an
       * unhandled condition) has occurred.  errval should should hold
       * the (positive) error code.
       */

      if (wrbytes != ((ssize_t)buflen))
        {
          ndbg("Write failed: wrbytes=%d errval=%d\n", wrbytes, errval);
          (void)ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 550, ' ', "Data send error !");
           ret = -errval;
           break;
        }

      /* Get the next file offset */

      pos += (off_t)wrbytes;
    }
    
errout_with_session:;
    close(session->fd);
    session->fd = -1;

    if (isnew && ret < 0)
      {
        (void)unlink(path);
      }

errout_with_data:;
    (void)ftpd_dataclose(session);

errout_with_path:
    free(abspath);

errout:
    return ret;
}

/****************************************************************************
 * Name: ftpd_listoption
 ****************************************************************************/

static uint8_t ftpd_listoption(FAR char **param)
{
  FAR char *ptr = *param;
  uint8_t ret = 0;

  while (*ptr == '-')
    {
      while (*ptr != '\0' && !isspace(*ptr))
        {
          switch (*ptr)
            {
              case 'a':
              case 'A':
                ret |= FTPD_LISTOPTION_A;
                break;

              case 'l':
              case 'L':
                ret |= FTPD_LISTOPTION_L;
                break;

              case 'f':
              case 'F':
                ret |= FTPD_LISTOPTION_F;
                break;

              case 'r':
              case 'R':
                ret |= FTPD_LISTOPTION_R;
                break;

              default:
                ret |= FTPD_LISTOPTION_UNKNOWN;
                break;
            }

          ptr++;
        }

      if (*ptr != '\0')
        {
          while (*ptr != '\0' && isspace(*ptr))
            {
              ptr++;
            }
        }
    }

  *param = ptr;
  return ret;
}

/****************************************************************************
 * Name: fptd_listscan
 ****************************************************************************/

static int ftpd_listbuffer(FAR struct ftpd_session_s *session, FAR char *path,
                           FAR struct stat *st, FAR char *buffer,
                           size_t buflen, unsigned int opton)
{
  FAR char *name;
  size_t offset = 0;

  name = basename(path);

  if ((opton & FTPD_LISTOPTION_L) != 0)
    {
      FAR const char *str;
      struct tm tm;
      time_t now;

      if (S_ISREG(st->st_mode) != 0)
        {
          str = "-";
        }
      else if (S_ISDIR(st->st_mode) != 0)
        {
          str = "d";
        }
      else if (S_ISCHR(st->st_mode) != 0)
        {
          str = "c";
        }
      else if (S_ISBLK(st->st_mode) != 0)
        {
          str = "b";
        }
      else if (S_ISFIFO(st->st_mode) != 0)
        {
          str = "p";
        }
      else if (S_ISLNK(st->st_mode) != 0)
        {
          str = "l";
        }
      else if (S_ISSOCK(st->st_mode) != 0)
        {
          str = "s";
        }
      else
        {
          str = "-";
        }

      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      /* User */

      str = ((st->st_mode & S_IRUSR) != 0) ? "r" : "-";
      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      str = ((st->st_mode & S_IWUSR) != 0) ? "w" : "-";
      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      if ((st->st_mode & S_ISUID) != 0 && (st->st_mode & S_IXUSR) != 0)
        {
          str = "s";
        }
      else if ((st->st_mode & S_ISUID) != 0)
        {
          str = "S";
        }
      else if ((st->st_mode & S_IXUSR) != 0)
        {
          str = "x";
        }
      else
        {
          str = "-";
        }

      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      /* group */

      str = ((st->st_mode & S_IRGRP) != 0) ? "r" : "-";
      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      str = ((st->st_mode & S_IWGRP) != 0) ? "w" : "-";
      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      if ((st->st_mode & S_ISGID) != 0 && (st->st_mode & S_IXGRP) != 0)
        {
          str = "s";
        }
      else if ((st->st_mode & S_ISGID) != 0)
        {
          str = "S";
        }
      else if ((st->st_mode & S_IXGRP) != 0)
        {
          str = "x";
        }
      else
        {
          str = "-";
        }

      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      /* other */

      str = ((st->st_mode & S_IROTH) != 0) ? "r" : "-";
      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      str = ((st->st_mode & S_IWOTH) != 0) ? "w" : "-";
      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

      if ((st->st_mode & S_ISVTX) != 0 && (st->st_mode & S_IXOTH) != 0)
        {
          str = "t";
        }
      else if ((st->st_mode & S_ISVTX) != 0)
        {
          str = "T";
        }
      else if ((st->st_mode & S_IXOTH) != 0)
        {
          str = "x";
        }
      else
        {
          str = "-";
        }

      offset += snprintf(&buffer[offset], buflen - offset, "%s", str);

#ifdef __NUTTX__
      /* Fake nlink, user id, and group id */

      offset += snprintf(&buffer[offset], buflen - offset, "%4u %8u %8u", 1, 1001, 512);
#else
      /* nlink */

      offset += snprintf(&buffer[offset], buflen - offset, "%4u", st->st_nlink);

      /* user id */

      offset += snprintf(&buffer[offset], buflen - offset, " %8u", st->st_uid);

      /* group id */

      offset += snprintf(&buffer[offset], buflen - offset, " %8u", st->st_gid);
#endif

      /* size */

      offset += snprintf(&buffer[offset], buflen - offset, " %8u", st->st_size);

      /* time */

      memcpy(&tm, localtime((FAR const time_t *)&st->st_mtime), sizeof(tm));
      offset += snprintf(&buffer[offset], buflen - offset, " %s %2u",
                         g_monthtab[tm.tm_mon], tm.tm_mday);
      now = time(0);
      if ((now - st->st_mtime) > (time_t)(60 * 60 * 24 * 180))
        {
          offset += snprintf(&buffer[offset], buflen - offset, " %5u",
                             tm.tm_year + 1900);
        }
      else
        {
          offset += snprintf(&buffer[offset], buflen - offset, " %02u:%02u",
                             tm.tm_hour, tm.tm_min);
        }
        
      /* basename */

      offset += snprintf(&buffer[offset], buflen - offset, " %s", name);

      /* linkname */

#ifndef __NUTTX__
      if (S_ISLNK(st->st_mode) != 0)
        {
          FAR char *temp;
          int namelen;

          temp = (FAR char *)malloc(PATH_MAX + 1);
          if (temp)
            {
              namelen = readlink(path, temp, PATH_MAX);
              if (namelen != (-1))\
                {
                  temp[namelen] = '\0';
                }

              offset += snprintf(&buffer[offset], buflen - offset, " -> %s", temp);
              free(temp);
            }
        }
#endif

      /* end */

      offset += snprintf(&buffer[offset], buflen - offset, "\r\n");
    }
  else
    {
      /* basename */

      offset += snprintf(&buffer[offset], buflen - offset, "%s\r\n", name);
    }

  return 0;
}

/****************************************************************************
 * Name: fptd_listscan
 ****************************************************************************/

static int fptd_listscan(FAR struct ftpd_session_s *session, FAR char *path,
                         unsigned int opton)
{
  FAR char *temp;
  DIR *dir;
  struct dirent *entry;
  struct stat st;
  int ret;

  ret = stat(path, &st);
  if (ret < 0)
    {
      return -errno;
    }

  if (!S_ISDIR(st.st_mode))
    {
      ret = ftpd_listbuffer(session, path, &st, session->data.buffer,
                            session->data.buflen, opton); 
      if (ret == 0)
        {
          ret = ftpd_response(session->data.sd, session->txtimeout,
                              "%s", (FAR char *)session->data.buffer);
        }

      return ret;
    }

  dir = opendir(path);
  if (!dir)
    {
      int errval = errno;
      ndbg("dir() failed\n", errval);
      return -errval;
    }

  for (;;)
    {
      entry = readdir(dir);
      if (!entry)
        {
          break;
        }

      if (entry->d_name[0] == '.')
        {
          if ((opton & FTPD_LISTOPTION_A) == 0)
            {
              continue;
            }
        }
 
      asprintf(&temp, "%s/%s", path, entry->d_name);
      if (!temp)
        {
          continue;
        }

      ret = stat(temp, &st);
      if (ret < 0)
        {
          free(temp);
          continue;
        }

      ret = ftpd_listbuffer(session, temp, &st, session->data.buffer,
                            session->data.buflen, opton); 
      if (ret >= 0)
        {
          ret = ftpd_response(session->data.sd, session->txtimeout,
                              "%s", session->data.buffer);
        }

      free(temp);
      if (ret < 0)
        {
          break;
        }
    }

  (void)closedir(dir);
  return ret;
}

/****************************************************************************
 * Name: ftpd_list
 ****************************************************************************/

static int ftpd_list(FAR struct ftpd_session_s *session, unsigned int opton)
{
  FAR char *abspath;
  int ret;

  ret = ftpd_getpath(session, session->param, &abspath, NULL);
  if (ret >= 0)
    {
      ret = fptd_listscan(session, abspath, opton);
      free(abspath);
    }

  return OK;
}

/****************************************************************************
 * Command Handlers
 ****************************************************************************/
/****************************************************************************
 * Name: ftpd_command_user
 ****************************************************************************/

static int ftpd_command_user(FAR struct ftpd_session_s *session)
{
  int ret;

  /* Clear session status */ 

  session->flags      = 0;
  session->restartpos = 0;

  /* Free session strings */

  if (session->user)
    {
      free(session->user);
      session->user = NULL;
    }

  if (session->renamefrom)
    {
      free(session->renamefrom);
      session->renamefrom = NULL;
    }

  /* Set up the new user */

  session->user = strdup(session->param);
  if (!session->user)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 451, ' ', "Memory exhausted !");
    }
  session->flags |= FTPD_SESSIONFLAG_USER;

  /* If there is no account information, then no login is required. */

  if (!session->head)
    {
      FAR char *home;

      home          = getenv("HOME");
      session->curr = NULL;
      session->home = strdup(!home ? "/" : home);
      session->work = strdup("/");

      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 230, ' ', "Login successful.");
      if (ret < 0)
        {
          session->curr = NULL;
        }
      return ret;
    }

  /* Try to login with no password.  This willwork if no password is
   * required for the account.
   */

  session->curr = ftpd_account_login(session, session->param, NULL);
  if (session->curr)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 230, ' ', "Login successful.");
      if (ret < 0)
        {
          session->curr = NULL;
        }
      return ret;
    }

  /* A password is required */

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt2, 331, ' ', "Password required for ",
                       session->user);
}

/****************************************************************************
 * Name: ftpd_command_pass
 ****************************************************************************/

static int ftpd_command_pass(FAR struct ftpd_session_s *session)
{
  int ret;

  if (!session->user)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 530, ' ', "Please login with USER !");
    }

  session->curr = ftpd_account_login(session, session->user, session->param);
  if (session->curr)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 230, ' ', "Login successful.");
      if (ret < 0)
        {
          session->curr = NULL;
        }
      return ret;
    }

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 530, ' ', "Login incorrect !");
}

/****************************************************************************
 * Name: ftpd_command_syst
 ****************************************************************************/

static int ftpd_command_syst(FAR struct ftpd_session_s *session)
{
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 215, ' ', "UNIX Type: L8");
}

/****************************************************************************
 * Name: ftpd_command_type
 ****************************************************************************/

static int ftpd_command_type(FAR struct ftpd_session_s *session)
{
  size_t parmlen = strlen(session->param);

  if (parmlen == 1)
    {
      switch (toupper(session->param[0]))
        {
        case 'A':
          {
            session->type = FTPD_SESSIONTYPE_A;
            return ftpd_response(session->cmd.sd, session->txtimeout,
                                 g_respfmt1, 200, ' ', "Type set to A");
          }

        case 'I':
          {
            session->type = FTPD_SESSIONTYPE_I;
            return ftpd_response(session->cmd.sd, session->txtimeout,
                                 g_respfmt1, 200, ' ', "Type set to I");
          }

        case 'L':
          {
            session->type = FTPD_SESSIONTYPE_L8;
            return ftpd_response(session->cmd.sd, session->txtimeout,
                                 g_respfmt1, 200, ' ',
                                 "Type set to L (byte size 8)");
          }

        default:
          {
            session->type = FTPD_SESSIONTYPE_NONE;
            return ftpd_response(session->cmd.sd, session->txtimeout,
                                 g_respfmt1, 501, ' ', "Type unknown !");
          }
        }
    }
  else if (parmlen == 3)
    {
      if (toupper(session->param[0]) == 'L' && session->param[1] == ' ')
        {
          if (session->param[2] == '8')
            {
              session->type = FTPD_SESSIONTYPE_L8;
              return ftpd_response(session->cmd.sd, session->txtimeout,
                                   g_respfmt1, 200, ' ', "Type set to L 8");
            }
          else
            {
              session->type = FTPD_SESSIONTYPE_NONE;
              return ftpd_response(session->cmd.sd, session->txtimeout,
                                   g_respfmt1, 504, ' ', "Byte size must be 8 !");
            }
        }
    }
    
  session->type = FTPD_SESSIONTYPE_NONE;
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 500, ' ', "TYPE not understood !");
}

/****************************************************************************
 * Name: ftpd_command_mode
 ****************************************************************************/

static int ftpd_command_mode(FAR struct ftpd_session_s *session)
{
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 502, ' ',
                       "MODE command not implemented !");
}

/****************************************************************************
 * Name: ftpd_command_abor
 ****************************************************************************/

static int ftpd_command_abor(FAR struct ftpd_session_s *session)
{
  (void)ftpd_dataclose(session);
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 426, ' ',
                       "Transfer aborted. Data connection closed.");
}

/****************************************************************************
 * Name: ftpd_command_quit
 ****************************************************************************/

static int ftpd_command_quit(FAR struct ftpd_session_s *session)
{
  (void)ftpd_response(session->cmd.sd, session->txtimeout,
                      g_respfmt1, 221, ' ', "Good-bye");

  /* Return a negative value to force the server to disconnect */

  return -1;
}

/****************************************************************************
 * Name: ftpd_command_noop
 ****************************************************************************/

static int ftpd_command_noop(FAR struct ftpd_session_s *session)
{
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 200, ' ',
                       "NOOP command successful");
}

/****************************************************************************
 * Name: ftpd_command_port
 ****************************************************************************/

static int ftpd_command_port(FAR struct ftpd_session_s *session)
{
  uint8_t value[6];
  unsigned int utemp;
  int temp;
  FAR char *str;
  int index;
  int ret;

  index = 0;
  while (index < 6)
    {
      /* Get the next value from the comma-delimited string */

      str = ftpd_strtok(true, ",", &session->param);
      if (*str == '\0')
        {
          break;
        }

      /* ftpd_strtok differs from the real strtok in that it does not NUL-
       * terminate the strings.
       */

      if (session->param[0] != '\0')
        {
          session->param[0] = '\0';
          session->param++;
        }

      /* Get the next value from the list */

      temp = atoi(str);
      if (temp < 0 || temp > 255)
        {
          ret = ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 501, ' ',
                              "Illegal PORT command");
          if (ret < 0)
            {
              ndbg("ftpd_response failed: %d\n", ret);
              return ret;
            }
        }

      value[index++] = (uint8_t)temp;
    }

  if (index < 6)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 501, ' ', "Illegal PORT command");
    }

  (void)ftpd_dataclose(session);

#if 1 /* Follow param */

  memset(&session->data.addr, 0, sizeof(session->data.addr));

  session->data.addr.in4.sin_family = AF_INET;

  utemp = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3]);
  session->data.addr.in4.sin_addr.s_addr = htonl((long)utemp);
    
  utemp = (value[4] << 8) | (value[5]);
  session->data.addr.in4.sin_port = htons((short)utemp);

#else /* Follow command socket address */

  session->data.addrlen = sizeof(session->data.addr);
  ret = getpeername(session->cmd.sd, (struct sockaddr *)&session->data.addr,
                   &session->data.addrlen);
  if (ret >= 0)
    {
      if (session->data.addr.ss.ss_family != AF_INET)
        {
          memset(&session->data.addr, 0, sizeof(session->data.addr));

          session->data.addr.in4.sin_family = AF_INET;
    
          utemp = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3]);
          session->data.addr.in4.sin_addr.s_addr = htonl(utemp);
        }
            
      utemp = (value[4] << 8) | (value[5]);
      session->data.addr.in4.sin_port = htons(utemp);
    }
  else
    {
      memset(&session->data.addr, 0, sizeof(session->data.addr));

      session->data.addr.in4.sin_family = AF_INET;
    
      utemp = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3]);
      session->data.addr.in4.sin_addr.s_addr = htonl(utemp);
    }

  utemp = (value[4] << 8) | (value[5]);
  session->data.addr.in4.sin_port = htons(utemp);
#endif

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 200, ' ',
                       "PORT command successful");
}

/****************************************************************************
 * Name: ftpd_command_eprt
 ****************************************************************************/

static int ftpd_command_eprt(FAR struct ftpd_session_s *session)
{
  FAR const char *str;
  FAR char *field[3];
  sa_family_t family;
  size_t left;
  size_t right;
  int count;
  int index;

  left = 0;
  right = strlen(session->param);

  if (right <= 0)
    {
      /* no message ? */

      (void)ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 502, ' ',
                          "EPRT command not implemented !");
      return -EINVAL;
    }
  right--;

  while (session->param[left] != '\0')
    {
      if (session->param[left] == '|')
        {
          left++;
          break;
        }
      left++;
    }

  if (right <= 0 || left > right)
    {
      /* Invalid format */

      (void)ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 502, ' ',
                          "EPRT command not implemented !");
      return -EINVAL;
    }
            
  count = 3;
  for (index = 0; index < count; index++)
    {
      field[index] = NULL;
    }
    
  str = (FAR const char *)&session->param[left];
  for (index = 0; index < count && *str != '\0'; index++)
    {
      field[index] = ftpd_strtok_alloc(true, ",|)", &str);
       if (!field[index])
         {
            break;
         }

       if (*str != '\0')
         {
            str++;
         }
    }

  if (index < count)
    {
      for (index = 0; index < count; index++)
        {
          if (field[index])
            {
              free(field[index]);
            }
        }

      (void)ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 502, ' ',
                          "EPRT command not implemented !");
        return -EINVAL;
    }

  (void)ftpd_dataclose(session);

  memset(&session->data.addr, 0, sizeof(session->data.addr));
  family = atoi(field[0]);
#ifndef CONFIG_NET_IPv6
  if (family == 1)
    {
      family = AF_INET;
        
      session->data.addr.in4.sin_family = family;
      (void)inet_pton(family, field[1], &session->data.addr.in4.sin_addr);
      session->data.addr.in4.sin_port = htons((short)atoi(field[2]));
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (family == 2)
    {
      family = AF_INET6;
        
      session->data.addr.in6.sin6_family = family;
      (void)inet_pton(family, field[1], &session->data.addr.in6.sin6_addr);
      session->data.addr.in6.sin6_port = htons((short)atoi(field[2]));
    }
  else
#endif
    {
      ndbg("Unrecognized family: %d\n", family);
      family = AF_UNSPEC;
    }
    
  for (index = 0;index < count;index++)
    {
      if (field[index])
        {
          free(field[index]);
        }
    }

  if (family == AF_UNSPEC)
    {
      ftpd_response(session->cmd.sd, session->txtimeout,
                    g_respfmt1, 502, ' ',
                    "EPRT command not implemented !");
      return -EINVAL;
    }

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 200, ' ', "EPRT command successful");
}

/****************************************************************************
 * Name: ftpd_command_pwd
 ****************************************************************************/

static int ftpd_command_pwd(FAR struct ftpd_session_s *session)
{
  FAR const char *workpath;

  if (!session->work)
    {
      workpath = "";
    }
  else
    {
      workpath = session->work;
    }

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       "%03u%c\"%s\" is current directory.\r\n",
                       257, ' ', workpath);
}

/****************************************************************************
 * Name: ftpd_command_cwd
 ****************************************************************************/

static int ftpd_command_cwd(FAR struct ftpd_session_s *session)
{
  return ftpd_changedir(session, session->param);
}

/****************************************************************************
 * Name: ftpd_command_cdup
 ****************************************************************************/

static int ftpd_command_cdup(FAR struct ftpd_session_s *session)
{
  return ftpd_changedir(session, g_cdup);
}

/****************************************************************************
 * Name: ftpd_command_rmd
 ****************************************************************************/

static int ftpd_command_rmd(FAR struct ftpd_session_s *session)
{
  FAR char *abspath;
  FAR char *workpath;
  int ret;

  ret = ftpd_getpath(session, session->param, &abspath, &workpath);
  if (ret < 0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not remove directory !");
    }

  if (strcmp(session->home, abspath) == 0)
    {
      free(abspath);
      free(workpath);

      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not remove home directory !");
    }

  if (strcmp(session->work, workpath) == 0)
    {
      free(abspath);
      free(workpath);

      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not remove current directory !");
    }

  ret = rmdir(abspath);
  if (ret < 0)
    {
      free(abspath);
      free(workpath);

      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not remove directory !");
    }
    
  free(abspath);
  free(workpath);

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 250, ' ',
                       "RMD command successful");
}

/****************************************************************************
 * Name: ftpd_command_mkd
 ****************************************************************************/

static int ftpd_command_mkd(FAR struct ftpd_session_s *session)
{
  FAR char *abspath;
  int ret;

  ret = ftpd_getpath(session, session->param, &abspath, NULL);
  if (ret < 0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not make directory !");
    }

  ret = mkdir(abspath, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
  if (ret < 0)
    {
      free(abspath);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "Can not make directory !");
    }
    
  free(abspath);
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 250, ' ', "MKD command successful");
}

/****************************************************************************
 * Name: ftpd_command_dele
 ****************************************************************************/

static int ftpd_command_dele(FAR struct ftpd_session_s *session)
{
  FAR char *abspath;
  FAR char *workpath;
  int ret;

  ret = ftpd_getpath(session, session->param, &abspath, &workpath);
  if (ret < 0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "Can not delete file !");
    }

  if (strcmp(session->home, abspath) == 0)
    {
      free(abspath);
      free(workpath);

      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not delete home directory !");
    }

  if (strcmp(session->work, workpath) == 0)
    {
      free(abspath);
      free(workpath);

      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ',
                           "Can not delete current directory !");
    }

  ret = unlink(abspath);
  if (ret < 0)
    {
      free(abspath);
      free(workpath);

      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "Can not delete file !");
    }
    
  free(abspath);
  free(workpath);

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 250, ' ', "DELE command successful");
}

/****************************************************************************
 * Name: ftpd_command_pasv
 ****************************************************************************/

static int ftpd_command_pasv(FAR struct ftpd_session_s *session)
{
  unsigned int value[6];
  unsigned int temp;
  int ret;

  (void)ftpd_dataclose(session);

  session->data.addrlen = sizeof(session->data.addr);

  session->data.sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (session->data.sd < 0)
    {
      (void)ftpd_dataclose(session);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 425, ' ', "PASV socket create fail !");
    }

  ret = getsockname(session->cmd.sd, (FAR struct sockaddr *)&session->data.addr,
                    &session->data.addrlen);
  if (ret < 0)
    {
      (void)ftpd_dataclose(session);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 425, ' ', "PASV getsockname fail !");
    }

#ifdef CONFIG_NET_IPv6
    if (session->data.addr.ss.ss_family == AF_INET6)
      {
        /* Convert ipv6 to ipv4 */

        if ((IN6_IS_ADDR_V4MAPPED(&session->data.addr.in6.sin6_addr) != 0) ||
            (IN6_IS_ADDR_V4COMPAT(&session->data.addr.in6.sin6_addr) != 0))
          {
            /* convert ipv6 to ipv4 */

            in_addr in4addr;

            in4addr.s_addr = session->data.addr.in6.sin6_addr.s6_addr32[3];

            memset(&session->data.addr, 0, sizeof(session->data.addr));
            session->data.addr.in4.sin_family = AF_INET;
            session->data.addr.in4.sin_addr.s_addr = in4addr.s_addr;
        }
    }
  else
#endif
#ifndef CONFIG_NET_IPv6
  if (session->data.addr.ss.ss_family == AF_INET)
    {
      /* Fixed to ipv4 */

      memset((FAR void *)(&session->data.addr), 0, sizeof(session->data.addr));
      session->data.addr.in4.sin_family = AF_INET;
      session->data.addr.in4.sin_addr.s_addr = htonl(INADDR_ANY);
    }
  else
#endif
    {
      ndbg("Unsupported family\n");
    }

  session->data.addr.in4.sin_port = 0;
  ret = bind(session->data.sd, (FAR const struct sockaddr *)&session->data.addr,
             session->data.addrlen);
  if (ret < 0)
    {
      (void)ftpd_dataclose(session);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 425, ' ', "PASV bind fail !");
    }

  ret = getsockname(session->data.sd, (FAR struct sockaddr *)&session->data.addr,
                    &session->data.addrlen);
  if (ret < 0)
    {
      (void)ftpd_dataclose(session);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 425, ' ', "PASV getsockname fail !");
    }

  ret = listen(session->data.sd, 1); 
  if (ret < 0)
    {
      (void)ftpd_dataclose(session);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 425, ' ', "PASV listen fail !");
    }

  if (ntohl(session->data.addr.in4.sin_addr.s_addr) == INADDR_ANY)
    {
      (void)ftpd_dataclose(session);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 425, ' ',
                           "Can not open passive connection");
    }

  temp = ntohl(session->data.addr.in4.sin_addr.s_addr);
  value[0] = (temp >> 24) & 0xff;
  value[1] = (temp >> 16) & 0xff;
  value[2] = (temp >> 8) & 0xff;
  value[3] = (temp) & 0xff;

  temp = (unsigned int)ntohs(session->data.addr.in4.sin_port);
  value[4] = (temp >> 8) & 0xff;
  value[5] = (temp) & 0xff;

  ret = ftpd_response(session->cmd.sd, session->txtimeout,
                      "%03u%cEntering passive mode (%u,%u,%u,%u,%u,%u).\r\n",
                      227, ' ',
                      value[0], value[1], value[2],
                      value[3], value[4], value[5]);
  if (ret < 0)
    {
      (void)ftpd_dataclose(session);
    }

  return ret;
}

/****************************************************************************
 * Name: ftpd_command_epsv
 ****************************************************************************/

static int ftpd_command_epsv(FAR struct ftpd_session_s *session)
{
  int ret;

  (void)ftpd_dataclose(session);

  session->data.addrlen = sizeof(session->data.addr);

#ifdef CONFIG_NET_IPv6
  session->data.sd = socket(PF_INET6, SOCK_STREAM, IPPROTO_TCP);
  if (session->data.sd < 0)
    {
      session->data.sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    }
  else
    {
#if defined(IPPROTO_IPV6) && defined(IPV6_V6ONLY)    
      int ipv6only = 0;
      (void)setsockopt(session->data.sd, IPPROTO_IPV6, IPV6_V6ONLY, &ipv6only, sizeof(ipv6only));
#endif        
    }
#else
  session->data.sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
#endif

  if (session->data.sd < 0)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 500, ' ', "EPSV socket create fail !");
      (void)ftpd_dataclose(session);
      return ret;
    }

  ret = getsockname(session->cmd.sd, (FAR struct sockaddr *)&session->data.addr,
                   &session->data.addrlen);
  if (ret < 0)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 500, ' ', "EPSV getsockname fail !");
      (void)ftpd_dataclose(session);
      return ret;
    }

#ifdef CONFIG_NET_IPv6
  if (session->data.addr.ss.ss_family == AF_INET6)
    {
      session->data.addr.in6.sin6_port = htons(0);
    }
  else if (session->data.addr.ss.ss_family == AF_INET)
    {
      session->data.addr.in4.sin_port = htons(0);
    }
#else
  session->data.addr.in4.sin_port = htons(0);
#endif    

  ret = bind(session->data.sd, (FAR const struct sockaddr *)&session->data.addr,
             session->data.addrlen);
  if (ret < 0)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 500, ' ', "EPSV bind fail !");
      (void)ftpd_dataclose(session);
      return ret;
    }

  ret = getsockname(session->data.sd, (FAR struct sockaddr *)&session->data.addr,
                    &session->data.addrlen);
  if (ret < 0)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 500, ' ', "EPSV getsockname fail !");
      (void)ftpd_dataclose(session);
      return ret;
    }

  ret = listen(session->data.sd, 1);
  if (ret < 0)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
            g_respfmt1, 500, ' ', "EPSV listen fail !");
      (void)ftpd_dataclose(session);
      return ret;
    }

#ifdef CONFIG_NET_IPv6
  if (session->data.addr.ss.ss_family == AF_INET6)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          "%03u%cEntering Extended Passive Mode (|||%u|).\r\n",
                          229, ' ',
                          ntohs(session->data.addr.in6.sin6_port));
      if (ret < 0)
        {
          (void)ftpd_dataclose(session);
          return ret;
        }
    }
  else
#else
  if (session->data.addr.ss.ss_family == AF_INET)
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          "%03u%cEntering Extended Passive Mode (|%u||%u|).\r\n",
                          229, ' ', 1,
                          ntohs(session->data.addr.in4.sin_port));
      if (ret < 0)
        {
          (void)ftpd_dataclose(session);
          return ret;
        }
    }
  else
#endif
    {
      ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 502, ' ',
                          "EPSV command not implemented !");
    }

  return ret;
}

/****************************************************************************
 * Name: ftpd_command_list
 ****************************************************************************/

static int ftpd_command_list(FAR struct ftpd_session_s *session)
{
  uint8_t opton = FTPD_LISTOPTION_L;
  int ret;

  ret = ftpd_dataopen(session);
  if (ret < 0)
    {
      return 0;
    }

  ret = ftpd_response(session->cmd.sd, session->txtimeout,
                      g_respfmt1, 150, ' ',
                      "Opening ASCII mode data connection for file list");
  if (ret < 0)
    {
      (void)ftpd_dataclose(session);
      return ret;
    }

  opton |= ftpd_listoption((char **)(&session->param));
  (void)ftpd_list(session, opton);

  ret = ftpd_response(session->cmd.sd, session->txtimeout,
                      g_respfmt1, 226, ' ', "Transfer complete");

  (void)ftpd_dataclose(session);
  return ret;
}

/****************************************************************************
 * Name: ftpd_command_nlst
 ****************************************************************************/

static int ftpd_command_nlst(FAR struct ftpd_session_s *session)
{
  uint8_t opton = 0;
  int ret;

  ret = ftpd_dataopen(session);
  if (ret < 0)
    {
      return 0;
    }

  ret = ftpd_response(session->cmd.sd, session->txtimeout,
                      g_respfmt1, 150, ' ',
                      "Opening ASCII mode data connection for file list");
  if (ret < 0)
    {
      (void)ftpd_dataclose(session);
      return ret;
    }

  opton |= ftpd_listoption((char **)(&session->param));
  (void)ftpd_list(session, opton);

  ret = ftpd_response(session->cmd.sd, session->txtimeout,
                      g_respfmt1, 226, ' ', "Transfer complete");

  (void)ftpd_dataclose(session);
  return ret;
}

/****************************************************************************
 * Name: ftpd_command_acct
 ****************************************************************************/

static int ftpd_command_acct(FAR struct ftpd_session_s *session)
{
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 502, ' ', "ACCT command not implemented !");
}

/****************************************************************************
 * Name: ftpd_command_size
 ****************************************************************************/

static int ftpd_command_size(FAR struct ftpd_session_s *session)
{
  FAR char *abspath;
  FAR char *path;
  struct stat st;
  FAR FILE *outstream;
  off_t offset;
  int ch;
  int status;
  int ret;

  ret = ftpd_getpath(session, session->param, &abspath, NULL);
  if (ret < 0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "Unknown size !");
    }
  path = abspath;
    
  ret = 0;
  switch (session->type)
    {
    case FTPD_SESSIONTYPE_NONE:
    case FTPD_SESSIONTYPE_L8:
    case FTPD_SESSIONTYPE_I:
      {
        status = stat(path, &st);
        if (status < 0)
          {
            ret = ftpd_response(session->cmd.sd, session->txtimeout,
                                g_respfmt2, 550, ' ', session->param,
                                ": not a regular file.");
          }
        else if (!S_ISREG(st.st_mode))
          {
             ret = ftpd_response(session->cmd.sd, session->txtimeout,
                                 g_respfmt2, 550, ' ', session->param,
                                 ": not a regular file.");
          }
        else
          { 
            ret = ftpd_response(session->cmd.sd, session->txtimeout,
                                "%03u%c%llu\r\n", 213, ' ', (unsigned long long)st.st_size);
          }
      }
      break;

    case FTPD_SESSIONTYPE_A: 
      {
        status = stat(path, &st);
        if (status < 0)
          {
            ret = ftpd_response(session->cmd.sd, session->txtimeout,
                                g_respfmt2, 550, ' ', session->param,
                                ": not a regular file.");
            if (ret < 0)
              {
                return ret;
              }
          }
        else if (!S_ISREG(st.st_mode))
          {
            ret = ftpd_response(session->cmd.sd, session->txtimeout,
                                g_respfmt2, 550, ' ', session->param,
                                ": not a regular file.");
            if (ret < 0)
              {
                return ret;
              }
          }


        outstream = fopen(path, "r");
        if (!outstream)
          {
            ret = ftpd_response(session->cmd.sd, session->txtimeout,
                                g_respfmt2, 550, ' ', session->param,
                                ": Can not open file !");
            if (ret < 0)
              {
                return ret;
              }
           }

        offset = 0;
        for (;;)
          {
            ch = getc(outstream);
            if (ch == EOF)
              {
                break;
              }
            else if (ch == 'c')
              {
                offset++;
              }
            offset++;
          }

        (void)fclose(outstream);
        ret = ftpd_response(session->cmd.sd, session->txtimeout,
                            "%03u%c%llu\r\n", 213, ' ', (unsigned long long)offset);
      }
      break;

    default:
      {
        ret = ftpd_response(session->cmd.sd, session->txtimeout,
                            g_respfmt1, 504, ' ', "SIZE not implemented for type");
      }
      break;
    }

  free(abspath);
  return ret;
}

/****************************************************************************
 * Name: ftpd_command_stru
 ****************************************************************************/

static int ftpd_command_stru(FAR struct ftpd_session_s *session)
{
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 502, ' ', "STRU command not implemented !");
}

/****************************************************************************
 * Name: 
 ****************************************************************************/

static int ftpd_command_rnfr(FAR struct ftpd_session_s *session)
{
  FAR char *abspath;
  FAR char *path;
  struct stat st;
  int ret;

  if (session->renamefrom)
    {
      free(session->renamefrom);
      session->renamefrom = NULL;
    }

  ret = ftpd_getpath(session, session->param, &abspath, NULL);
  if (ret < 0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "RNFR error !");
    }
  path = abspath;

  ret = stat(path, &st);
  if (ret < 0)
    {
      free(abspath);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt2, 550, ' ', session->param,
                           ": No such file or directory.");
    }
    
  session->renamefrom = abspath;
  session->flags |= FTPD_SESSIONFLAG_RENAMEFROM;

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 350, ' ', "RNFR successful");
}

/****************************************************************************
 * Name: ftpd_command_rnto
 ****************************************************************************/

static int ftpd_command_rnto(FAR struct ftpd_session_s *session)
{
  FAR char *abspath;
  int ret;
    
  if (!session->renamefrom)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "RNTO error !");
    }

  ret = ftpd_getpath(session, session->param, &abspath, NULL);
  if (ret < 0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "RNTO error !");
    }
 
  ret = rename(session->renamefrom, abspath);
  if (ret < 0)
    {
      free(abspath);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt2, 550, ' ', session->param,
                           ": Rename error.");
    }

  free(abspath);
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 250, ' ', "Rename successful");
}

/****************************************************************************
 * Name: ftpd_command_retr
 ****************************************************************************/

static int ftpd_command_retr(FAR struct ftpd_session_s *session)
{
    return ftpd_stream(session, 0);
}

/****************************************************************************
 * Name: ftpd_command_stor
 ****************************************************************************/

static int ftpd_command_stor(FAR struct ftpd_session_s *session)
{
    return ftpd_stream(session, 1);
}

/****************************************************************************
 * Name: ftpd_command_appe
 ****************************************************************************/

static int ftpd_command_appe(FAR struct ftpd_session_s *session)
{
    return ftpd_stream(session, 2);
}

/****************************************************************************
 * Name: ftpd_command_rest
 ****************************************************************************/

static int ftpd_command_rest(FAR struct ftpd_session_s *session)
{
#ifdef CONFIG_HAVE_LONG_LONG
    session->restartpos = (off_t)atoll(session->param);
#else    
    session->restartpos = (off_t)atoi(session->param);
#endif
    session->flags |= FTPD_SESSIONFLAG_RESTARTPOS;

    return ftpd_response(session->cmd.sd, session->txtimeout,
                         g_respfmt1, 320, ' ', "Restart position ready");
}

/****************************************************************************
 * Name: ftpd_command_mdtm
 ****************************************************************************/

static int ftpd_command_mdtm(FAR struct ftpd_session_s *session)
{
  FAR char *abspath;
  FAR char *path;
  struct stat st;
  struct tm tm;
  int ret;

  ret = ftpd_getpath(session, session->param, &abspath, NULL);
  if (ret <0)
    {
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 550, ' ', "Unknown size !");
    }
  path = abspath;

  ret = stat(path, &st);
  if (ret < 0)
    {
      free(abspath);
      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt2, 550, ' ', session->param,
                           ": not a plain file.");
    }

    if (!S_ISREG(st.st_mode))
      {
        free(abspath);
        return ftpd_response(session->cmd.sd, session->txtimeout,
                             g_respfmt2, 550, ' ', session->param,
                             ": not a plain file.");
    }
    
  free(abspath);
  
  memcpy(&tm, gmtime(&st.st_mtime), sizeof(tm)); 
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       "%03u%c%04u%02u%02u%02u%02u%02u\r\n", 213, ' ',
                       tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                       tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/****************************************************************************
 * Name: ftpd_command_opts
 ****************************************************************************/

static int ftpd_command_opts(FAR struct ftpd_session_s *session)
{
  FAR char *str;
  FAR char *option;
  FAR char *value;
  bool remote = false;
  bool local = false;

  /* token: name and value */

  str = session->param;
  option = ftpd_strtok(true, " \t", &str);

  /* Unlike the "real" strtok, ftpd_strtok does not NUL-terminate
   * the returned string.
   */

  if (*str != '\0')
    {
      *str = '\0';
      str++;
    }
  value = str;

  if (strcasecmp(option, "UTF8") == 0 || strcasecmp(option, "UTF-8") == 0)
    {
      FAR char *lang;

      if (value[0] == '\0' || strcasecmp(value, "ON") == 0 ||
          strcasecmp(value, "ENABLE") == 0 || strcasecmp(value, "TRUE") == 0)
        {
            remote = true;
        }
        else {
            remote = false;
        }

      lang = getenv("LANG");
      if (lang)
        {
          if (strcasestr(lang, "UTF8") || strcasestr(lang, "UTF-8"))
            {
              local = true;
            }
          else
            {
              local = false;
            }
        }
#if 1 /*  OPTION: UTF-8 is default */
      else
        {
          local = true;
        }
#endif

      if (remote != local)
        {
          return ftpd_response(session->cmd.sd, session->txtimeout,
                               g_respfmt1, 504, ' ', "UIF-8 disabled");
        }

      return ftpd_response(session->cmd.sd, session->txtimeout,
                           g_respfmt1, 200, ' ', "OK, UTF-8 enabled");
    }

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       "%03u%c%s%s%s\r\n", 501, ' ', "OPTS: ", option,
                       " not understood");
}

/****************************************************************************
 * Name: ftpd_command_site
 ****************************************************************************/

static int ftpd_command_site(FAR struct ftpd_session_s *session)
{
  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt1, 502, ' ', "SITE command not implemented !");
}

/****************************************************************************
 * Name: ftpd_command_help
 ****************************************************************************/

static int ftpd_command_help(FAR struct ftpd_session_s *session)
{
  int index;
  int ret;

  index = 0;
  while (g_ftpdhelp[index])
    {
      if (index == 0 || !g_ftpdhelp[index + 1])
        {
          ret = ftpd_response(session->cmd.sd, session->txtimeout,
                              g_respfmt1, 214,
                              !g_ftpdhelp[index + 1] ? ' ' : '-',
                              g_ftpdhelp[index]);
        }
      else
        {
          ret = ftpd_response(session->cmd.sd, session->txtimeout,
                              "%c%s\r\n", ' ', g_ftpdhelp[index]);
        }

      if (ret < 0)
        {
          return ret;
        }

      index++;
    }

  return OK;
}

/****************************************************************************
 * Name: ftpd_command
 ****************************************************************************/

static int ftpd_command(FAR struct ftpd_session_s *session)
{
  int index = 0;

  /* Search the command table for a matching command */
 
  for (index = 0; g_ftpdcmdtab[index].command; index++)
    {
      /* Does the command string match this entry? */

      if (strcmp(session->command, g_ftpdcmdtab[index].command) == 0)
        {
          /* Yes.. is a login required to execute this command? */
 
          if ((g_ftpdcmdtab[index].flags & FTPD_CMDFLAG_LOGIN) != 0)
            {
              /* Yes... Check if the user is logged in */

              if (!session->curr && session->head)
                {
                  return ftpd_response(session->cmd.sd, session->txtimeout,
                                       g_respfmt1, 530, ' ',
                                       "Please login with USER and PASS !");
                }
            }

          /* Check if there is a handler for the command */

          if (g_ftpdcmdtab[index].handler)
            {
              /* Yess.. invoke the command handler. */

              return g_ftpdcmdtab[index].handler(session);
            }

          /* No... this command is not in the command table.  Break out of
           * the loop and send the 500 message.
           */

          break;
        }
    }

  /* There is nothing in the command table matching this command */

  return ftpd_response(session->cmd.sd, session->txtimeout,
                       g_respfmt2, 500, ' ', session->command,
                       " not understood");
}

/****************************************************************************
 * Worker Thread
 ****************************************************************************/
/****************************************************************************
 * Name: ftpd_startworker
 ****************************************************************************/

static int ftpd_startworker(pthread_startroutine_t handler, FAR void *arg,
                            size_t stacksize)
{
  pthread_t threadid;
  pthread_attr_t attr;
  int ret;

  /* Initialize the thread attributes */

  ret = pthread_attr_init(&attr);
  if (ret != 0)
    {
      ndbg("pthread_attr_init() failed: %d\n", ret);
      goto errout;
    }

  /* The set the thread stack size */

  ret = pthread_attr_setstacksize(&attr, stacksize);
  if (ret != 0)
    {
      ndbg("pthread_attr_setstacksize() failed: %d\n", ret);
      goto errout_with_attr;
    }

  /* And create the thread */

  ret = pthread_create(&threadid, &attr, handler, arg);
  if (ret != 0)
    {
      ndbg("pthread_create() failed: %d\n", ret);
      goto errout_with_attr;
    }

  /* Put the thread in the detached stated */

  ret = pthread_detach(threadid);
  if (ret != 0)
    {
      ndbg("pthread_detach() failed: %d\n", ret);
    }

errout_with_attr:
  pthread_attr_destroy(&attr);
errout:
  return -ret;
}

/****************************************************************************
 * Name: ftpd_freesession
 ****************************************************************************/

static void ftpd_freesession(FAR struct ftpd_session_s *session)
{
  /* Free resources */

  if (session->renamefrom)
    {
      free(session->renamefrom);
    }

  if (session->work)
    {
      free(session->work);
    }

  if (session->home)
    {
      free(session->home);
    }

  if (session->user)
    {
      free(session->user);
    }

  if (session->fd < 0)
    {
      close(session->fd);
    }

  if (session->data.buffer)
    {
      free(session->data.buffer);
    }

  (void)ftpd_dataclose(session);

  if (session->cmd.buffer)
    {
      free(session->cmd.buffer);
    }

  if (session->cmd.sd <0)
    {
      close(session->cmd.sd);
    }

  free(session);
}

/****************************************************************************
 * Name: ftpd_workersetup
 ****************************************************************************/

static void ftpd_workersetup(FAR struct ftpd_session_s *session)
{
#if defined(CONFIG_NET_HAVE_IPTOS) || defined(CONFIG_NET_HAVE_OOBINLINE)
  int temp;
#endif
#ifdef CONFIG_NET_HAVE_SOLINGER
  struct linger ling;
#endif

#ifdef CONFIG_NET_HAVE_IPTOS
  temp = IPTOS_LOWDELAY;
  (void)setsockopt(session->cmd.sd, IPPROTO_IP, IP_TOS, &temp, sizeof(temp));
#endif

#ifdef CONFIG_NET_HAVE_OOBINLINE
  temp = 1;
  (void)setsockopt(session->cmd.sd, SOL_SOCKET, SO_OOBINLINE, &temp, sizeof(temp));
#endif

#ifdef CONFIG_NET_HAVE_SOLINGER
  (void)memset(&ling, 0, sizeof(ling));
  ling.l_onoff = 1;
  ling.l_linger = 4;
  (void)setsockopt(session->cmd.sd, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling));
#endif
}

/****************************************************************************
 * Name: ftpd_worker
 ****************************************************************************/

static FAR void *ftpd_worker(FAR void *arg)
{
  FAR struct ftpd_session_s *session = (FAR struct ftpd_session_s *)arg;
  ssize_t recvbytes;
  size_t offset;
  uint8_t ch;
  int ret;

  nvdbg("Worker started\n");
  DEBUGASSERT(session);

  /* Configure the session sockets */

  ftpd_workersetup(session);

  /* Send the welcoming message */

  ret = ftpd_response(session->cmd.sd, session->txtimeout,
                          g_respfmt1, 220, ' ', CONFIG_FTPD_SERVERID);
  if (ret < 0)
    {
      ndbg("ftpd_response() failed: %d\n", ret);
      ftpd_freesession(session);
      return NULL;
    }

  /* Then loop processing FTP commands */

 for (;;)
    {
      /* Receive the next command */

      recvbytes = ftpd_recv(session->cmd.sd, session->cmd.buffer,
                            session->cmd.buflen - 1, session->rxtimeout);

      /* recbytes < 0 is a receive failure (posibily a timeout); 
       * recbytes == 0 indicates that we have lost the connection.
       */

      if (recvbytes <= 0)
        {
          /* Break out of the server loop */

          break;
        }

      /* Make sure that the recevied string is NUL terminated */

      session->cmd.buffer[recvbytes] = '\0';
       
      /* TELNET protocol (RFC854)
       *   IAC   255(FFH) interpret as command:
       *   IP    244(F4H) interrupt process--permanently
       *   DM    242(F2H) data mark--for connect. cleaning
       */

      offset = 0;
      while (recvbytes > 0)
        {
          ch = session->cmd.buffer[offset];
            if (ch != 0xff && ch != 0xf4 && ch != 0xf2)
              {
                break;
              }

          (void)ftpd_send(session->cmd.sd, &session->cmd.buffer[offset], 1, session->txtimeout);

          offset++;
          recvbytes--;
        }

      /* Just continue if there was nothing of interest in the packet */

      if (recvbytes <= 0)
        {
          continue;
        }

      /* Make command message */
 
      session->command = &session->cmd.buffer[offset];
      while (session->cmd.buffer[offset] != '\0')
        {
          if (session->cmd.buffer[offset] == '\r' &&
              session->cmd.buffer[offset + ((ssize_t)1)] == '\n')
            {
              session->cmd.buffer[offset] = '\0';
              break;
            }
          offset++;    
        }

      /* Parse command and param tokens */

      session->param   = session->command;
      session->command = ftpd_strtok(true, " \t", &session->param);

      /* Unlike the "real" strtok, ftpd_strtok does not NUL-terminate
       * the returned string.
       */

      if (session->param[0] != '\0')
        {
          session->param[0] = '\0';
          session->param++;
        }

      /* Dispatch the FTP command */

      ret = ftpd_command(session);
      if (ret < 0)
        {
          ndbg("Disconnected by the command handler: %d\n", ret);
          break;
        }
    }

  ftpd_freesession(session);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpd_open
 *
 * Description:
 *   Create an instance of the FTPD server and return a handle that can be
 *   used to run the server.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that can be used to reference
 *   the server instance.
 *
 ****************************************************************************/

FTPD_SESSION ftpd_open(void)
{
  FAR struct ftpd_server_s *server;

  server = ftpd_openserver(21);
  if (!server)
    {
      server = ftpd_openserver(2211);
    }

  return (FTPD_SESSION)server;
}

/****************************************************************************
 * Name: ftpd_adduser
 *
 * Description:
 *   Add one FTP user.
 *
 * Input Parameters:
 *    handle - A handle previously returned by ftpd_open
 *    accountflags - The characteristics of this user (see FTPD_ACCOUNTFLAGS_*
 *      definitions).
 *    user - The user login name. May be NULL indicating that no login is
 *      required.
 *    passwd - The user password.  May be NULL indicating that no password
 *      is required.
 *    home - The user home directory. May be NULL.
 *
 * Returned Value:
 *   Zero is returned on success.  A negated errno value is return on
 *   failure.
 *
 ****************************************************************************/

int ftpd_adduser(FTPD_SESSION handle, uint8_t accountflags,
                 FAR const char *user, FAR const char *passwd,
                 FAR const char *home)
{
  FAR struct ftpd_server_s  *server;
  FAR struct ftpd_account_s *newaccount;
  int ret;

  DEBUGASSERT(handle);

  newaccount = ftpd_account_new(user, accountflags);
  if (!newaccount)
    {
      ndbg("Failed to allocte memory to the account\n");
      ret = -ENOMEM;
      goto errout;
    }

  ret = ftpd_account_setpassword(newaccount, passwd);
  if (ret < 0)
    {
      ndbg("ftpd_account_setpassword failed: %d\n", ret);
      goto errout_with_account;
    }
 
  ret = ftpd_account_sethome(newaccount, home);
  if (ret < 0)
    {
      ndbg("ftpd_account_sethome failed: %d\n", ret);
      goto errout_with_account;
    }

  server = (FAR struct ftpd_server_s *)handle;
  ret = ftpd_account_add(server, newaccount);
  if (ret < 0)
    {
      ndbg("ftpd_account_add failed: %d\n", ret);
      goto errout_with_account;
    }

  return OK;

errout_with_account:
  ftpd_account_free(newaccount);
  
errout:
  return ret;
}

/****************************************************************************
 * Name: ftpd_session
 *
 * Description:
 *   Execute the FTPD server.  This thread does not return until either (1)
 *   the timeout expires with no connection, (2) some other error occurs, or
 *   (2) a connection was accepted and an FTP worker thread was started to
 *   service the session.
 *
 * Input Parameters:
 *   handle - A handle previously returned by ftpd_open
 *   timeout - A time in milliseconds to wait for a connection. If this
 *     time elapses with no connected, the -ETIMEDOUT error will be returned.
 *
 * Returned Value:
 *   Zero is returned if the FTP worker was started.  On failure, a negated
 *   errno value is returned to indicate why the servier terminated.
 *   -ETIMEDOUT indicates that the user-provided timeout elapsed with no
 *   connection.
 *
 ****************************************************************************/

int ftpd_session(FTPD_SESSION handle, int timeout)
{
  FAR struct ftpd_server_s  *server;
  FAR struct ftpd_session_s *session;
  int ret;

  DEBUGASSERT(handle);

  server = (FAR struct ftpd_server_s *)handle;

  /* Allocate a session */

  session = (FAR struct ftpd_session_s *)zalloc(sizeof(struct ftpd_session_s));
  if (!session)
    {
      ndbg("Failed to allocate session\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Initialize the session */

  session->server       = server;
  session->head         = server->head;
  session->curr         = NULL;
  session->flags        = 0;
  session->txtimeout    = -1; 
  session->rxtimeout    = -1; 
  session->cmd.sd       = (int)(-1);
  session->cmd.addrlen  = (socklen_t)sizeof(session->cmd.addr);
  session->cmd.buflen   = (size_t)CONFIG_FTPD_CMDBUFFERSIZE;
  session->cmd.buffer   = NULL;
  session->command      = NULL;
  session->param        = NULL;
  session->data.sd      = -1;
  session->data.addrlen = sizeof(session->data.addr);
  session->data.buflen  = CONFIG_FTPD_DATABUFFERSIZE;
  session->data.buffer  = NULL;
  session->restartpos   = 0;
  session->fd           = -1;
  session->user         = NULL;
  session->type         = FTPD_SESSIONTYPE_NONE;
  session->home         = NULL;
  session->work         = NULL;
  session->renamefrom   = NULL;

  /* Allocate a command buffer */

  session->cmd.buffer = (FAR char *)malloc(session->cmd.buflen);
  if (!session->cmd.buffer)
    {
      ndbg("Failed to allocate command buffer\n");
      ret = -ENOMEM;
      goto errout_with_session;
    }

  /* Allocate a data buffer */
  
  session->data.buffer = (FAR char *)malloc(session->data.buflen);
  if (!session->data.buffer)
    {
      ndbg("Failed to allocate data buffer\n");
      ret = -ENOMEM;
      goto errout_with_session;
    }

  /* Accept a connection */

  session->cmd.sd = ftpd_accept(server->sd, (FAR void *)&session->cmd.addr,
                                &session->cmd.addrlen, timeout);
  if (session->cmd.sd < 0)
    {
      /* Only report interesting, infrequent errors (not the common timeout) */

#ifdef CONFIG_DEBUG_NET
      if (session->cmd.sd != -ETIMEDOUT)
        {
          ndbg("ftpd_accept() failed: %d\n", session->cmd.sd);
        }
#endif
      ret = session->cmd.sd;
      goto errout_with_session;
    }

  /* And create a worker thread to service the session */

  ret = ftpd_startworker(ftpd_worker, (FAR void *)session,
                         CONFIG_FTPD_WORKERSTACKSIZE);
  if (ret < 0)
    {
      ndbg("ftpd_startworker() failed: %d\n", ret);
      goto errout_with_session;
    }

  /* Successfully connected an launched the worker thread */

  return 0;

errout_with_session:
  ftpd_freesession(session);
errout:
  return ret;
}

/****************************************************************************
 * Name: ftpd_close
 *
 * Description:
 *   Close and destroy the handle created by ftpd_open.
 *
 * Input Parameters:
 *   handle - A handle previously returned by ftpd_open
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ftpd_close(FTPD_SESSION handle)
{
  struct ftpd_server_s *server;
  DEBUGASSERT(handle);

  server = (struct ftpd_server_s *)handle;
  ftpd_account_free(server->head);

  if (server->sd >= 0)
    {
      close(server->sd);
      server->sd = -1;
    }

  free(server);
}

