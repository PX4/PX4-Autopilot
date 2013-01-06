//***************************************************************************
// examples/cxxtest/main.cxx
//
//   Copyright (C) 2012 Gregory Nutt. All rights reserved.
//   Author: Qiang Yu, http://rgmp.sourceforge.net/wiki/index.php/Main_Page
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
#include <nuttx/arch.h>

#include <iostream>
#include <vector>
#include <map>
#include <stdexcept>
#include <cassert>

using namespace std;

//***************************************************************************
// Definitions
//***************************************************************************

//***************************************************************************
// Private Classes
//***************************************************************************

class Base
{
public:
  virtual void printBase(void) {};
};

class Extend : public Base
{
public:
  void printExtend(void)
  {
    cout << "extend" << endl;
  }
};

//***************************************************************************
// Private Data
//***************************************************************************

//***************************************************************************
// Public Functions
//***************************************************************************

//***************************************************************************
// Name: test_iostream
//***************************************************************************/

static void test_iostream(void)
{
  cout << "test iostream===========================" << endl;
  cout << "Hello, this is only a test" << endl;
  cout << "Print an int: "  <<  190  <<  endl;
  cout <<  "Print a char: "  <<  'd'  <<  endl;

#if 0
  int a;
  string s;

  cout << "Please type in an int:" << endl;
  cin >> a;
  cout << "You type in: " << a << endl;
  cout << "Please type in a string:" << endl;
  cin >> s;
  cout << "You type in: " << s << endl;
#endif
}

//***************************************************************************
// Name: test_stl
//***************************************************************************/

static void test_stl(void)
{
  cout << "test vector=============================" << endl;

  vector<int> v1;
  assert(v1.empty());

  v1.push_back(1);
  assert(!v1.empty());

  v1.push_back(2);
  v1.push_back(3);
  v1.push_back(4);
  assert(v1.size() == 4);

  v1.pop_back();
  assert(v1.size() == 3);

  cout << "v1=" << v1[0] << ' ' << v1[1] << ' ' << v1[2] << endl;
  assert(v1[2] == 3);

  vector<int> v2 = v1;
  assert(v2 == v1);

  string words[4] = {"Hello", "World", "Good", "Luck"};
  vector<string> v3(words, words + 4);
  vector<string>::iterator it;
  for (it = v3.begin(); it != v3.end(); it++)
    {
      cout << *it << ' ';
    }

  cout << endl;
  assert(v3[1] == "World");

  cout << "test map================================" << endl;

  map<int,string> m1;
  m1[12] = "Hello";
  m1[24] = "World";
  assert(m1.size() == 2);
  assert(m1[24] == "World");
}

//***************************************************************************
// Name: test_rtti
//***************************************************************************/

static void test_rtti(void)
{
  cout << "test rtti===============================" << endl;
  Base *a = new Base();
  Base *b = new Extend();
  assert(a);
  assert(b);

  Extend *t = dynamic_cast<Extend *>(a);
  assert(t == NULL);

  t = dynamic_cast<Extend *>(b);
  assert(t);
  t->printExtend();

  delete a;
  delete b;
}

//***************************************************************************
// Name: test_exception
//***************************************************************************/

#ifdef CONFIG_UCLIBCXX_EXCEPTION
static void test_exception(void)
{
  cout << "test exception==========================" << endl;
  try
  {
    throw runtime_error("runtime error");
  }

  catch (runtime_error &e)
  {
    cout << "Catch exception: " << e.what() << endl;
  }
}
#endif

//***************************************************************************
// Public Functions
//***************************************************************************

//***************************************************************************
// Name: cxxtest_main
//***************************************************************************/

extern "C"
{
  int cxxtest_main(int argc, char *argv[])
  {
    // If C++ initialization for static constructors is supported, then do
    // that first

#ifdef CONFIG_HAVE_CXXINITIALIZE
    up_cxxinitialize();
#endif

    test_iostream();
    test_stl();
    test_rtti();
#ifdef CONFIG_UCLIBCXX_EXCEPTION
    test_exception();
#endif
    
    return 0;
  }
}

