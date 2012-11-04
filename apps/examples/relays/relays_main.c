/****************************************************************************
 * examples/relays/relays_main.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Darcy Gong
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
#include <nuttx/arch.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#ifdef CONFIG_ARCH_RELAYS

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_RELAYS_NRELAYS
#  define CONFIG_EXAMPLES_RELAYS_NRELAYS  2
#endif

 /****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: relays_main
 ****************************************************************************/

int relays_main(int argc, char *argv[])
{
  char *stat = NULL;
  char *no = NULL;
  bool badarg = false;
  bool set_stat = false;
  uint32_t r_stat;
  int option;
  int n = -1;
  int ret = -1;
  int i;

  while ((option = getopt(argc, argv, ":n:")) != ERROR)
    {
      switch (option)
        {
          case 'n':
            no = optarg;
            n = atoi(no);
            break;

          case ':':
            badarg = true;
            break;

          case '?':
          default:
            badarg = true;
            break;
        }
    }

  if (badarg)
    {
      printf("usage: relays [ -n <relay id> ] <switch-status>\n");
      return -1;
    }
  
  if (optind == argc - 1)
    {
      stat = argv[optind];
      set_stat = (!strcmp(stat,"on") || !strcmp(stat,"ON")) ? true : false ;
    }
  
  up_relaysinit();
    
  if (n >= 0)
    {
      printf("set RELAY ID %d to %s\n", n , set_stat ? "ON" : "OFF");
      relays_setstat(n,set_stat);
    }
  else
    {
      r_stat = 0;
      for (i = 0; i < CONFIG_EXAMPLES_RELAYS_NRELAYS; i++)
        {
          printf("set RELAY ID %d to %s\n", i , set_stat ? "ON" : "OFF");
          r_stat |= (set_stat ? 1 : 0) << i;
        }

      relays_setstats(r_stat);
    }

  return 0;
}

#endif /* CONFIG_ARCH_RELAYS */
