/*	Copyright (C) 2004 Garrett A. Kajmowicz

	This file is part of the uClibc++ Library.

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <setjmp.h>

#ifndef __STD_HEADER_CSETJMP
#define __STD_HEADER_CSETJMP 1


//From GCC Header files
#undef longjmp

// Adhere to section 17.4.1.2 clause 5 of ISO 14882:1998
#ifndef setjmp
#define setjmp(env) setjmp (env)
#endif

//Mine again


namespace std{
	using ::longjmp;
	using ::jmp_buf;
}


#endif

