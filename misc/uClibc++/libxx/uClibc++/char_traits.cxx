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

#define __UCLIBCXX_COMPILE_CHAR_TRAITS__ 1


#include <basic_definitions>
#include <char_traits>

namespace std{

_UCXXEXPORT const char_traits<char>::char_type* char_traits<char>::find(const char_type* s, int n, const char_type& a){
	for(int i=0; i < n; i++){
		if(eq(s[i], a)){
			return (s+i);
		}
	}
	return 0;
}

_UCXXEXPORT bool char_traits<char>::eq(const char_type& c1, const char_type& c2){
	if(strncmp(&c1, &c2, 1) == 0){
		return true;
	}
	return false;
}

_UCXXEXPORT char_traits<char>::char_type char_traits<char>::to_char_type(const int_type & i){
	if(i > 0 && i <= 255){
		return (char)(unsigned char)i;
	}

	//Out of range
	return 0;
}



#ifdef __UCLIBCXX_HAS_WCHAR__

_UCXXEXPORT const char_traits<wchar_t>::char_type* char_traits<wchar_t>::find(const char_type* s, int n, const char_type& a){
	for(int i=0; i < n; i++){
		if(eq(s[i], a)){
			return (s+i);
		}
	}
	return 0;
}

#endif

}
