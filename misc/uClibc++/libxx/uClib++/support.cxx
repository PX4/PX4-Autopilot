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

#include <support>

extern "C" void *__cxa_allocate_exception(size_t thrown_size){
	void * retval;

//	The amount of data needed is the size of the object *PLUS*
//	the size of the header.  The header is of struct __cxa_exception
//	The address needs to be adjusted because the pointer we return
//	should not point to the start of the memory, but to the point
//	where the object being thrown actually starts
//
	retval = malloc(thrown_size + sizeof(__cxa_exception));

//	Check to see that we actuall allocated memory
	if(retval == 0){
		std::terminate();
	}

	//Need to do a typecast to char* otherwize we are doing math with
	//a void* which makes the compiler cranky (Like me)
	return ((char *)retval + sizeof(__cxa_exception));
}

extern "C" void __cxa_free_exception(void *thrown_exception){



}

extern "C" void __cxa_throw (void *thrown_exception, void *info,void (*dest) (void *) ){


}

