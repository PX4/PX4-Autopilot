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

#define __UCLIBCXX_COMPILE_STREAMBUF__ 1

#include <streambuf>

namespace std{

#ifdef __UCLIBCXX_EXPAND_STREAMBUF_CHAR__

#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT streambuf::basic_streambuf();
	template _UCXXEXPORT streambuf::~basic_streambuf();

#endif

	template _UCXXEXPORT locale streambuf::pubimbue(const locale &loc);
	template _UCXXEXPORT streamsize streambuf::in_avail();
	template _UCXXEXPORT streambuf::int_type streambuf::sbumpc();
	template _UCXXEXPORT streambuf::int_type streambuf::snextc();
	template _UCXXEXPORT streambuf::int_type streambuf::sgetc();
	template _UCXXEXPORT streambuf::int_type streambuf::sputbackc(char_type c);
	template _UCXXEXPORT streambuf::int_type streambuf::sungetc();
	template _UCXXEXPORT streambuf::int_type streambuf::sputc(char_type c);

#endif


}


