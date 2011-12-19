/****************************************************************************
 * hello/hello.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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

#include <stdio.h>
#include <stdlib.h>


void memtest(void)
{
	char *p;
	int   i, j;
	
	for (i=0; i<1000; i++) {
		p = malloc(40000);
		if (!p) {
			printf("No memory\n");
			break;
		}
		for (j=0; j<40000; j++) p[j] = 0;
		free(p);
	}
}


/** Example of a standalone application
 */ 
int hello_main(int argc, char *argv[])
{
	int i;
		
	printf("Hello Builtin Application\n"
		   "Found argc=%d arguments and are as follows:\n", argc);
		   
	// note that stdout is bufferred and that fflush() and is called on exit.
	fflush(stdout);
		   
	for (i=0; i<argc; i++) 
		printf("%s\n", argv[i]);
		
	//memtest();
	
	return 0;
}
