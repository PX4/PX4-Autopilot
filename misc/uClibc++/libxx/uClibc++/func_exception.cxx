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

#include <exception>
#include <func_exception>
#include <stdexcept>
#include <cstdlib>

namespace std{

#ifdef __UCLIBCXX_EXCEPTION_SUPPORT__

_UCXXEXPORT void __throw_bad_alloc(){
	throw bad_alloc();
}

_UCXXEXPORT void __throw_out_of_range( const char * message){
	if(message == 0){
		throw out_of_range();
	}
	throw out_of_range(message);
}

_UCXXEXPORT void __throw_overflow_error( const char * message){
	if(message == 0){
		throw overflow_error();
	}
	throw overflow_error(message);
}

_UCXXEXPORT void __throw_length_error(const char * message){
	if(message == 0){
		throw length_error();
	}
	throw length_error(message);
}

_UCXXEXPORT void __throw_invalid_argument(const char * message){
	if(message == 0){
		throw invalid_argument();
	}
	throw invalid_argument(message);
}

#else

_UCXXEXPORT void __throw_bad_alloc(){
	abort();
}

_UCXXEXPORT void __throw_out_of_range( const char * ){
	abort();
}

_UCXXEXPORT void __throw_overflow_error( const char * ){
	abort();
}

_UCXXEXPORT void __throw_length_error(const char * ){
	abort();
}

_UCXXEXPORT void __throw_invalid_argument(const char *){
	abort();
}

#endif



}
