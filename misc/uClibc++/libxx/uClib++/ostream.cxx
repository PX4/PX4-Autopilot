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

#define __UCLIBCXX_COMPILE_OSTREAM__ 1

#include <ostream>

namespace std{
	

#ifdef __UCLIBCXX_EXPAND_OSTREAM_CHAR__
	
#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__
	template _UCXXEXPORT ostream::~basic_ostream();
#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT ostream & ostream::flush();

	template _UCXXEXPORT ostream & ostream::operator<<(bool n);
	template _UCXXEXPORT ostream & ostream::operator<<(short int n);
	template _UCXXEXPORT ostream & ostream::operator<<(unsigned short int n);
	template _UCXXEXPORT ostream & ostream::operator<<(int n);
	template _UCXXEXPORT ostream & ostream::operator<<(unsigned int n);
	template _UCXXEXPORT ostream & ostream::operator<<(long n);
	template _UCXXEXPORT ostream & ostream::operator<<(unsigned long n);
	template _UCXXEXPORT ostream & ostream::operator<<(float f);
	template _UCXXEXPORT ostream & ostream::operator<<(double f);
	template _UCXXEXPORT ostream & ostream::operator<<(long double f);
	template _UCXXEXPORT ostream & ostream::operator<<(void* p);
	template _UCXXEXPORT ostream & ostream::operator<<(basic_streambuf<char, char_traits<char> >* sb);

#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT ostream::sentry::sentry(ostream & os);
	template _UCXXEXPORT ostream::sentry::~sentry();

#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT ostream & endl(ostream & os);
	template _UCXXEXPORT ostream & flush(ostream & os);
	template _UCXXEXPORT ostream & operator<<(ostream & out, char c);
	template _UCXXEXPORT ostream & operator<<(ostream & out, const char* c);
	template _UCXXEXPORT ostream & operator<<(ostream & out, unsigned char c);
	template _UCXXEXPORT ostream & operator<<(ostream & out, const unsigned char* c);

#endif


}
