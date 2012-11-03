/* Copyright (C) 2004 Garrett A. Kajmowicz
 *
 * This file is part of the uClibc++ Library.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <new>
#include <cstdlib>

#include <basic_definitions>

#ifdef CONFIG_UCLIBCXX_EXCEPTION
#  include <func_exception>
#else
#  include <exception>
#endif

_UCXXEXPORT void *operator new[](std::size_t numBytes) throw(std::bad_alloc)
{
  // C++ stardard 5.3.4.8 requires that a valid pointer be returned for
  // a call to new(0). Thus:

  if (numBytes == 0)
  {
    numBytes = 1;
  }

  // Allocate the memory

  void *p = malloc(numBytes);
  if (p == 0)
  {
#ifdef CONFIG_UCLIBCXX_EXCEPTION
    std::__throw_bad_alloc();
#else
    std::terminate();
#endif
  }

  return p;
}
