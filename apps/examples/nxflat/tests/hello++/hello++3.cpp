/////////////////////////////////////////////////////////////////////////////
// examples/nxflat/tests/hello++/hello++3.c
//
//   Copyright (C) 2009 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
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
/////////////////////////////////////////////////////////////////////////////
//
// This is an another trivial version of "Hello, World" design.  It illustrates
//
// - Building a C++ program to use the C library and stdio
// - Basic class creation with virtual methods.
// - Static constructor and destructors (in main program only)
// - NO Streams
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Included Files
/////////////////////////////////////////////////////////////////////////////

#include <cstdio>

/////////////////////////////////////////////////////////////////////////////
// Classes
/////////////////////////////////////////////////////////////////////////////

class CThingSayer
{
  const char *szWhatToSay;
public:
  CThingSayer(void);
  virtual ~CThingSayer(void);
  virtual void Initialize(const char *czSayThis);
  virtual void SayThing(void);
};

// A static instance of the CThingSayer class.  This instance MUST
// be constructed by the system BEFORE the program is started at
// main() and must be destructed by the system AFTER the main()
// returns to the system

static CThingSayer MyThingSayer;

// These are implementations of the methods of the CThingSayer class

CThingSayer::CThingSayer(void)
{
  printf("CThingSayer::CThingSayer: I am!\n");
  szWhatToSay = (const char*)NULL;
}

CThingSayer::~CThingSayer(void)
{
  printf("CThingSayer::~CThingSayer: I cease to be\n");
  if (szWhatToSay)
    {
      printf("CThingSayer::~CThingSayer: I will never say '%s' again\n",
  	     szWhatToSay);
    }
  szWhatToSay = (const char*)NULL;
}

void CThingSayer::Initialize(const char *czSayThis)
{
  printf("CThingSayer::Initialize: When told, I will say '%s'\n",
	 czSayThis);
  szWhatToSay = czSayThis;
}

void CThingSayer::SayThing(void)
{
  printf("CThingSayer::SayThing: I am now saying '%s'\n", szWhatToSay);
}

/////////////////////////////////////////////////////////////////////////////
// Public Functions
/////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  // We should see the message from constructor, CThingSayer::CThingSayer(),
  // BEFORE we see the following messages.  That is proof that the
  // C++ static initializer is working

  printf("main: Started.  MyThingSayer should already exist\n");

  // Tell MyThingSayer that "Hello, World!" is the string to be said

  printf("main: Calling MyThingSayer.Initialize\n");;
  MyThingSayer.Initialize("Hello, World!");

  // Tell MyThingSayer to say the thing we told it to say

  printf("main: Calling MyThingSayer.SayThing\n");;
  MyThingSayer.SayThing();

  // We are finished, return.  We should see the message from the
  // destructor, CThingSayer::~CThingSayer(), AFTER we see the following
  // message.  That is proof that the C++ static destructor logic
  // is working

  printf("main: Returning.  MyThingSayer should be destroyed\n");;
  return 0;
}
