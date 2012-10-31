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

#define __UCLIBCXX_COMPILE_VECTOR__ 1


#include <vector>

namespace std{


#ifdef __UCLIBCXX_EXPAND_VECTOR_BASIC__

#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT vector<char, allocator<char> >::vector(const allocator<char>& al);
	template _UCXXEXPORT vector<char, allocator<char> >::vector(size_type n, const char & value, const allocator<char> & al);

	template _UCXXEXPORT vector<char, allocator<char> >::~vector();
	template _UCXXEXPORT vector<unsigned char, allocator<unsigned char> >::~vector();

#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT void vector<char, allocator<char> >::reserve(size_type n);
	template _UCXXEXPORT void vector<unsigned char, allocator<unsigned char> >::reserve(size_type n);
	template _UCXXEXPORT void vector<short int, allocator<short int> >::reserve(size_type n);
	template _UCXXEXPORT void vector<unsigned short int, allocator<unsigned short int> >::reserve(size_type n);
	template _UCXXEXPORT void vector<int, allocator<int> >::reserve(size_type n);
	template _UCXXEXPORT void vector<unsigned int, allocator<unsigned int> >::reserve(size_type n);
	template _UCXXEXPORT void vector<long int, allocator<long int> >::reserve(size_type n);
	template _UCXXEXPORT void vector<unsigned long int, allocator<unsigned long int> >::reserve(size_type n);
	template _UCXXEXPORT void vector<float, allocator<float> >::reserve(size_type n);
	template _UCXXEXPORT void vector<double, allocator<double> >::reserve(size_type n);
	template _UCXXEXPORT void vector<bool, allocator<bool> >::reserve(size_type n);

	template _UCXXEXPORT void vector<char, allocator<char> >::resize(size_type sz, const char & c);
	template _UCXXEXPORT void vector<unsigned char, allocator<unsigned char> >::resize(size_type sz, const unsigned char & c);
	template _UCXXEXPORT void vector<short int, allocator<short int> >::resize(size_type sz, const short & c);
	template _UCXXEXPORT void vector<unsigned short int, allocator<unsigned short int> >
		::resize(size_type sz, const unsigned short int & c);
	template _UCXXEXPORT void vector<int, allocator<int> >::resize(size_type sz, const int & c);
	template _UCXXEXPORT void vector<unsigned int, allocator<unsigned int> >::resize(size_type sz, const unsigned int & c);
	template _UCXXEXPORT void vector<long int, allocator<long int> >::resize(size_type sz, const long int & c);
	template _UCXXEXPORT void vector<unsigned long int, allocator<unsigned long int> >::
		resize(size_type sz, const unsigned long int & c);
	template _UCXXEXPORT void vector<float, allocator<float> >::resize(size_type sz, const float & c);
	template _UCXXEXPORT void vector<double, allocator<double> >::resize(size_type sz, const double & c);
	template _UCXXEXPORT void vector<bool, allocator<bool> >::resize(size_type sz, const bool & c);

#elif defined __UCLIBCXX_EXPAND_STRING_CHAR__


#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__
	template _UCXXEXPORT vector<char, allocator<char> >::vector(const allocator<char>& al);
	template _UCXXEXPORT vector<char, allocator<char> >::vector(size_type n, const char & value, const allocator<char> & al);
	template _UCXXEXPORT vector<char, allocator<char> >::~vector();
#endif // __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT void vector<char, allocator<char> >::reserve(size_type n);
	template _UCXXEXPORT void vector<char, allocator<char> >::resize(size_type sz, const char & c);

#endif




}
