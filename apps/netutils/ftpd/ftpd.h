/****************************************************************************
 * apps/include/ftpd.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_NETUTILS_FTPD_FTPD_H
#define __APPS_NETUTILS_FTPD_FTPD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>

#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* FPTD Definitions *********************************************************/

#define FTPD_SESSIONFLAG_USER       (1 << 0)  /* Session has a user */
#define FTPD_SESSIONFLAG_RESTARTPOS (1 << 1)  /* Session has a restart position */
#define FTPD_SESSIONFLAG_RENAMEFROM (1 << 2)  /* Session has a rename from string */

#define FTPD_LISTOPTION_A           (1 << 0)  /* List option 'A' */
#define FTPD_LISTOPTION_L           (1 << 1)  /* List option 'L' */
#define FTPD_LISTOPTION_F           (1 << 2)  /* List option 'F' */
#define FTPD_LISTOPTION_R           (1 << 3)  /* List option 'R' */
#define FTPD_LISTOPTION_UNKNOWN     (1 << 7)  /* Unknown list option */

#define FTPD_CMDFLAG_LOGIN          (1 << 0)  /* Command requires login */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This enumerates the type of each session */

enum ftpd_sessiontype_e
{
  FTPD_SESSIONTYPE_NONE = 0,
  FTPD_SESSIONTYPE_A,
  FTPD_SESSIONTYPE_I,
  FTPD_SESSIONTYPE_L8
};

struct ftpd_pathnode_s
{
  struct ftpd_pathnode_s    *flink;
  struct ftpd_pathnode_s    *blink;
  bool                       ignore;
  FAR char                  *name;
};

union ftpd_sockaddr_u
{
  uint8_t                    raw[sizeof(struct sockaddr_storage)];
  struct sockaddr_storage    ss;
  struct sockaddr            sa;
#ifdef CONFIG_NET_IPv6
  struct sockaddr_in6        in6;
#else
  struct sockaddr_in         in4;
#endif
};

/* This structure describes on account */

struct ftpd_account_s
{
  struct ftpd_account_s     *blink;
  struct ftpd_account_s     *flink;
  uint8_t                    flags;    /* See FTPD_ACCOUNTFLAG_* definitions */
  FAR char                  *user;     /* User name */
  FAR char                  *password; /* Un-encrypted password */
  FAR char                  *home;     /* Home directory path */
};

/* This structures describes an FTP session a list of associated accounts */

struct ftpd_server_s
{
  int                        sd;     /* Listen socket descriptor */
  union ftpd_sockaddr_u      addr;   /* Listen address */
  struct ftpd_account_s     *head;   /* Head of a list of accounts */
  struct ftpd_account_s     *tail;   /* Tail of a list of accounts */
};

struct ftpd_stream_s
{
  int                        sd;      /* Socket descriptor */
  union ftpd_sockaddr_u      addr;    /* Network address */
  socklen_t                  addrlen; /* Length of the address */
  size_t                     buflen;  /* Length of the buffer */
  char                      *buffer;  /* Pointer to the buffer */
};

struct ftpd_session_s
{
  FAR struct ftpd_server_s  *server;
  FAR struct ftpd_account_s *head;
  FAR struct ftpd_account_s *curr;
  uint8_t                    flags;   /* See TPD_SESSIONFLAG_* definitions */
  int                        rxtimeout;
  int                        txtimeout;

  /* Command */

  struct ftpd_stream_s       cmd;
  FAR char                  *command;
  FAR char                  *param;

  /* Data */

  struct ftpd_stream_s       data;
  off_t                      restartpos;

  /* File */

  int fd;

  /* Current user */

  FAR char                  *user;
  uint8_t                    type;    /* See enum ftpd_sessiontype_e */
  FAR char                  *home;
  FAR char                  *work;
  FAR char                  *renamefrom;
};

typedef int (*ftpd_cmdhandler_t)(struct ftpd_session_s *);

struct ftpd_cmd_s
{
  FAR const char            *command;  /* The command string */
  ftpd_cmdhandler_t          handler;  /* The function that handles the command */
  uint8_t                    flags;    /* See FTPD_CMDFLAGS_* definitions */
};

/* Used to maintain a list of protocol names */

struct ftpd_protocol_s
{
  FAR const char            *name;
  int value;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __APPS_NETUTILS_FTPD_FTPD_H */
