//***************************************************************************
// examples/helloxx/main.c
//
//   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>
#include <nuttx/init.h>
#include <cstdio>
#include <debug.h>

//***************************************************************************
// Definitions
//***************************************************************************

//***************************************************************************
// Private Classes
//***************************************************************************

class CHelloWorld
{
  public:
    CHelloWorld(void) : mSecret(42) { lldbg("Constructor\n"); };
    ~CHelloWorld(void) { lldbg("Destructor\n"); };

    bool HelloWorld(void)
    {
        if (mSecret != 42)
          {
            printf("CHelloWorld::HelloWorld: CONSTRUCTION FAILED!\n");
            return false;
          }
        else
          {
            printf("CHelloWorld::HelloWorld: Hello, World!!\n");
            return true;
          }
    };

  private:
    int mSecret;
};

//***************************************************************************
// Private Data
//***************************************************************************

#ifndef CONFIG_EXAMPLES_HELLOXX_NOSTATICCONST 
static CHelloWorld g_HelloWorld;
#endif

//***************************************************************************
// Public Functions
//***************************************************************************

//***************************************************************************
// user_start
//***************************************************************************

/****************************************************************************
 * Name: user_start/nxhello_main
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_HELLOXX_BUILTIN
extern "C" int helloxx_main(int argc, char *argv[]);
#  define MAIN_NAME   helloxx_main
#  define MAIN_STRING "helloxx_main: "
#else
#  define MAIN_NAME   user_start
#  define MAIN_STRING "user_start: "
#endif

int MAIN_NAME(int argc, char *argv[])
{
#ifndef CONFIG_EXAMPLES_HELLOXX_NOSTACKCONST
  CHelloWorld HelloWorld;
#endif
  CHelloWorld *pHelloWorld = new CHelloWorld;

  printf(MAIN_STRING "Saying hello from the dynamically constructed instance\n");
  pHelloWorld->HelloWorld();

#ifndef CONFIG_EXAMPLES_HELLOXX_NOSTACKCONST
  printf(MAIN_STRING "Saying hello from the instance constructed on the stack\n");
  HelloWorld.HelloWorld();
#endif

#ifndef CONFIG_EXAMPLES_HELLOXX_NOSTATICCONST
  printf(MAIN_STRING "Saying hello from the statically constructed instance\n");
  g_HelloWorld.HelloWorld();
#endif

  delete pHelloWorld;
  return 0;
}

