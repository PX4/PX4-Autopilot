/*  Copyright (C) 2006 Garrett A. Kajmowicz
 *
 *  This file is part of the uClibc++ Library.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, version 2.1
 * of the License.
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
#include <cstring>
#include <func_exception>

// This is a system-specific header which does all of the error-handling
// management

#include <unwind-cxx.h>

namespace __cxxabiv1
{
  extern "C" void *__cxa_allocate_exception(std::size_t thrown_size) throw()
  {
    void *retval;

    // The amount of data needed is the size of the object *PLUS*
    // the size of the header.  The header is of struct __cxa_exception
    // The address needs to be adjusted because the pointer we return
    // should not point to the start of the memory, but to the point
    // where the object being thrown actually starts

    retval = malloc (thrown_size + sizeof(__cxa_exception));

    // Check to see that we actuall allocated memory

    if (0 == retval)
      {
        std::terminate();
      }

    // Need to do a typecast to char* otherwize we are doing math with
    // a void* which makes the compiler cranky (Like me)

    memset (retval, 0, sizeof(__cxa_exception));
    return (void *)((unsigned char *)retval + sizeof(__cxa_exception));
  }

  extern "C" void __cxa_free_exception(void *vptr) throw()
  {
    free( (char *)(vptr) - sizeof(__cxa_exception) );
  }

  extern "C" __cxa_dependent_exception *__cxa_allocate_dependent_exception() throw()
  {
    __cxa_dependent_exception *retval;

    // The amount of data needed is the size of the object *PLUS*
    // the size of the header.  The header is of struct __cxa_exception
    // The address needs to be adjusted because the pointer we return
    // should not point to the start of the memory, but to the point
    // where the object being thrown actually starts

    retval = static_cast<__cxa_dependent_exception*>(malloc (sizeof(__cxa_dependent_exception)));

    // Check to see that we actuall allocated memory

    if (0 == retval)
      {
        std::terminate();
      }

    memset (retval, 0, sizeof(__cxa_dependent_exception));
    return retval;
  }

  extern "C" void __cxa_free_dependent_exception(__cxa_dependent_exception *vptr) throw()
  {
    free( (char *)(vptr) );
  }

  extern "C" void __cxa_throw(void *thrown_exception, std::type_info *tinfo, void (*dest) (void *))
  {
  }
}
