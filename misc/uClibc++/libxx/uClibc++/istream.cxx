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

#define __UCLIBCXX_COMPILE_ISTREAM__ 1

#include <istream>

namespace std{

#ifdef __UCLIBCXX_EXPAND_ISTREAM_CHAR__

	template <> _UCXXEXPORT string _readToken<char, char_traits<char> >(istream & stream)
	{
		string temp;
		char_traits<char>::int_type c;
		while(true){
			c = stream.rdbuf()->sgetc();
			if(c != char_traits<char>::eof() && isspace(c) == false){
				stream.rdbuf()->sbumpc();
				temp.append(1, char_traits<char>::to_char_type(c));
			}else{
				break;
			}
		}
		if (temp.size() == 0)
			stream.setstate(ios_base::eofbit|ios_base::failbit);

		return temp;
        }

	template _UCXXEXPORT istream::int_type istream::get();
	template _UCXXEXPORT istream & istream::get(char &c);

	template _UCXXEXPORT istream & istream::operator>>(bool &n);
	template _UCXXEXPORT istream & istream::operator>>(short &n);
	template _UCXXEXPORT istream & istream::operator>>(unsigned short &n);
	template _UCXXEXPORT istream & istream::operator>>(int &n);
	template _UCXXEXPORT istream & istream::operator>>(unsigned int &n);
	template _UCXXEXPORT istream & istream::operator>>(long unsigned &n);
	template _UCXXEXPORT istream & istream::operator>>(long int &n);
	template _UCXXEXPORT istream & istream::operator>>(void *& p);
	template _UCXXEXPORT istream & operator>>(istream & is, char & c);

#ifdef CONFIG_HAVE_FLOAT
	template _UCXXEXPORT istream & istream::operator>>(float &f);
	template _UCXXEXPORT istream & istream::operator>>(double &f);
	template _UCXXEXPORT istream & istream::operator>>(long double &f);
#endif

	template _UCXXEXPORT void __skipws(basic_istream<char, char_traits<char> >& is);

#endif


}

