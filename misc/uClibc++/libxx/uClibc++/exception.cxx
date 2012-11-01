/*  Copyright (C) 2004 Garrett A. Kajmowicz
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

#include <exception>

// We can't do this yet because gcc is too stupid to be able to handle
// different implementations of exception class.

#undef CONFIG_UCLIBCXX_EXCEPTION

#ifdef CONFIG_UCLIBCXX_EXCEPTION

namespace std
{
  _UCXXEXPORT static char * __std_exception_what_value = "exception";

  // We are providing our own versions to be sneaky

  _UCXXEXPORT exception::~exception() throw()
  {
    // Empty function
  }

  _UCXXEXPORT const char *exception::what() const throw()
  {
    return __std_exception_what_value;
  }

  _UCXXEXPORT bad_exception::~bad_exception() throw()
  {
  }
}

#endif
