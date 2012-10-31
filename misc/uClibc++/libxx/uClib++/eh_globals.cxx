/*	Copyright (C) 2006 Garrett A. Kajmowicz

	This file is part of the uClibc++ Library.

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation, version 2.1
	of the License.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <cstdlib>
#include <cstring>
#include <func_exception>

//This is a system-specific header which does all of the error-handling management
#include <unwind-cxx.h>

//The following functionality is derived from reading of the GNU libstdc++ code and making it...simple


namespace __cxxabiv1{

static __UCLIBCXX_TLS __cxa_eh_globals eh_globals;

extern "C" __cxa_eh_globals* __cxa_get_globals() throw(){
	return &eh_globals;
}

extern "C" __cxa_eh_globals* __cxa_get_globals_fast() throw(){
	return &eh_globals;
}

}
