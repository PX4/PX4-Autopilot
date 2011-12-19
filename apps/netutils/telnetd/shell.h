/****************************************************************************
 * netutils/telnetd/shell.h
 * Interface for the Contiki shell.
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2003, Adam Dunkels.
 *   All rights reserved.
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

/* Some of the functions declared in this file must be implemented as
 * a shell back-end in the architecture specific files of a Contiki
 * port.
 */

#ifndef __SHELL_H__
#define __SHELL_H__

/* Initialize the shell.
 *
 * Called when the shell front-end process starts. This function may
 * be used to start listening for signals.
 */

void shell_init(void *handle);

/* Start the shell back-end.
 *
 * Called by the front-end when a new shell is started.
 */

void shell_start(void *handle);

/* Process a shell command.
 *
 * This function will be called by the shell GUI / telnet server whan
 * a command has been entered that should be processed by the shell
 * back-end.
 *
 * command The command to be processed.
 */

void shell_input(void *handle, char *command);

/* Quit the shell. */

void shell_quit(void *handle, char *);

/* Print a string to the shell window.
 *
 * This function is implemented by the shell GUI / telnet server and
 * can be called by the shell back-end to output a string in the
 * shell window. The string is automatically appended with a linebreak.
 */

void shell_output(void *handle, const char *fmt, ...);

/* Print a prompt to the shell window.
 *
 * This function can be used by the shell back-end to print out a
 * prompt to the shell window.
 *
 */

void shell_prompt(void *handle, char *prompt);

#endif /* __SHELL_H__ */
