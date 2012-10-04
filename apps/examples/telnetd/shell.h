/****************************************************************************
 * apps/examples/telnetd/shell.h
 * Interface for the Contiki shell.
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_TELNETD_SHELL_H
#define __APPS_EXAMPLES_TELNETD_SHELL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_EXAMPLES_TELNETD_DAEMONPRIO - Priority of the Telnet daemon.
 *   Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_EXAMPLES_TELNETD_DAEMONSTACKSIZE - Stack size allocated for the
 *   Telnet daemon. Default: 2048
 * CONFIG_EXAMPLES_TELNETD_CLIENTPRIO- Priority of the Telnet client.
 *   Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_EXAMPLES_TELNETD_CLIENTSTACKSIZE - Stack size allocated for the
 *   Telnet client. Default: 2048
 * CONFIG_EXAMPLES_TELNETD_NOMAC - If the hardware has no MAC address of its
 *   own, define this =y to provide a bogus address for testing.
 * CONFIG_EXAMPLES_TELNETD_IPADDR - The target IP address.  Default 10.0.0.2
 * CONFIG_EXAMPLES_TELNETD_DRIPADDR - The default router address. Default
 *   10.0.0.1
 * CONFIG_EXAMPLES_TELNETD_NETMASK - The network mask.  Default: 255.255.255.0
 */

#ifndef CONFIG_EXAMPLES_TELNETD_DAEMONPRIO
#  define CONFIG_EXAMPLES_TELNETD_DAEMONPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_TELNETD_DAEMONSTACKSIZE
#  define CONFIG_EXAMPLES_TELNETD_DAEMONSTACKSIZE 2048
#endif

#ifndef CONFIG_EXAMPLES_TELNETD_CLIENTPRIO
#  define CONFIG_EXAMPLES_TELNETD_CLIENTPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_TELNETD_CLIENTSTACKSIZE
#  define CONFIG_EXAMPLES_TELNETD_CLIENTSTACKSIZE 2048
#endif

#ifndef CONFIG_EXAMPLES_TELNETD_IPADDR
#  define CONFIG_EXAMPLES_TELNETD_IPADDR 0x0a000002
#endif
#ifndef CONFIG_EXAMPLES_TELNETD_DRIPADDR
#  define CONFIG_EXAMPLES_TELNETD_DRIPADDR 0x0a000002
#endif
#ifndef CONFIG_EXAMPLES_TELNETD_NETMASK
#  define CONFIG_EXAMPLES_TELNETD_NETMASK 0xffffff00
#endif

/* Other definitions ********************************************************/

#define SHELL_PROMPT "uIP 1.0> "

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_EXAMPLES_TELNETD_SHELL_H */
