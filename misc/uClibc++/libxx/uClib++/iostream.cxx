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

#define __UCLIBCXX_COMPILE_IOSTREAM__ 1

#include <iostream>

namespace std{

#ifdef __UCLIBCXX_EXPAND_OSTREAM_CHAR__
#ifdef __UCLIBCXX_EXPAND_ISTREAM_CHAR__

	template _UCXXEXPORT basic_iostream<char, char_traits<char> >::
		basic_iostream(basic_streambuf<char, char_traits<char> >* sb);
	template _UCXXEXPORT basic_iostream<char, char_traits<char> >::~basic_iostream();

#endif
#endif

}


