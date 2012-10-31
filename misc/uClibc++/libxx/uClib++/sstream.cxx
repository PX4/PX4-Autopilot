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

#define __UCLIBCXX_COMPILE_SSTREAM__ 1

#include <sstream>

namespace std{

#ifdef __UCLIBCXX_EXPAND_SSTREAM_CHAR__

	typedef char_traits<char> tr_ch;
	typedef basic_stringbuf<char, tr_ch, allocator<char> > char_stringbuf;

#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT char_stringbuf::basic_stringbuf(ios_base::openmode which);
	template _UCXXEXPORT char_stringbuf::~basic_stringbuf();

#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT basic_string<char, char_traits<char>, allocator<char> > char_stringbuf::str() const;
	template _UCXXEXPORT char_stringbuf::int_type char_stringbuf::pbackfail(char_stringbuf::int_type c);
	template _UCXXEXPORT char_stringbuf::int_type char_stringbuf::overflow(char_stringbuf::int_type c);
	template _UCXXEXPORT char_stringbuf::pos_type
		char_stringbuf::seekoff(char_stringbuf::off_type, ios_base::seekdir, ios_base::openmode);
	template _UCXXEXPORT char_stringbuf::int_type char_stringbuf::underflow ();
	template _UCXXEXPORT streamsize char_stringbuf::xsputn(const char* s, streamsize n);

#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT basic_stringstream<char, tr_ch, allocator<char> >::basic_stringstream(ios_base::openmode which);
	template _UCXXEXPORT basic_istringstream<char, tr_ch, allocator<char> >::~basic_istringstream();
	template _UCXXEXPORT basic_ostringstream<char, tr_ch, allocator<char> >::~basic_ostringstream();
	template _UCXXEXPORT basic_stringstream<char, tr_ch, allocator<char> >::~basic_stringstream();

#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

#endif

}


