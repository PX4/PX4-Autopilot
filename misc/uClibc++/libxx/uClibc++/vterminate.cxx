/* Copyright (C) 2012 Gregory Nutt
 *
 * This file is part of the uClibc++ Library.
 *
 * A replacement for __gnu_cxx::terminate
 */

#include <basic_definitions>
#include <cstdlib>
#include <cunistd>
#include <debug.h>

// This is a brain-dead replacement for __gnu_cxx::__verbose_terminate_handler

namespace __gnu_cxx
{
  void __verbose_terminate_handler()
  {
     ldbg("PID %d: Terminating...\n", getpid());
     abort();
  }
}
