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

#include <cstdlib>
#include <exception>

namespace std
{
  _UCXXEXPORT static terminate_handler __terminate_handler = abort;
  _UCXXEXPORT static unexpected_handler __unexpected_handler = terminate;

  // Takes a new handler function as an argument, returns the old function.

  _UCXXEXPORT terminate_handler set_terminate(terminate_handler func) throw()
  {
    terminate_handler old = __terminate_handler;
    __terminate_handler = func;
    return old;
  }

  /** The runtime will call this function if %exception handling must be
   *  abandoned for any reason.
   */

  _UCXXEXPORT void terminate(void) throw()
  {
    if (__terminate_handler)
      {
        __terminate_handler();
      }

    abort();
  }

  _UCXXEXPORT terminate_handler set_unexpected(unexpected_handler func) throw()
  {
    unexpected_handler old = __unexpected_handler;
    __unexpected_handler = func;
    return old;
  }

  /** The runtime will call this function if an %exception is thrown which
   *  violates the function's %exception specification.
   */
 
  _UCXXEXPORT void unexpected(void) throw()
  {
    if (__unexpected_handler)
      {
        __unexpected_handler();
      }

    terminate();
  }
}
