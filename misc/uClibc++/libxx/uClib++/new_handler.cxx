/*	Copyright (C) 2005 Garrett A. Kajmowicz

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

#include <new>

const std::nothrow_t std::nothrow = { };

//Name selected to be compatable with g++ code
std::new_handler __new_handler;

_UCXXEXPORT std::new_handler std::set_new_handler(std::new_handler new_p) throw(){
	std::new_handler retval = __new_handler;
	__new_handler = new_p;
	return retval;
}
