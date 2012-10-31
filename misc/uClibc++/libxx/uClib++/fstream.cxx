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

#define __UCLIBCXX_COMPILE_FSTREAM__ 1

#include <fstream>

namespace std{

#ifdef __UCLIBCXX_EXPAND_FSTREAM_CHAR__

#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT filebuf::basic_filebuf();
	template _UCXXEXPORT filebuf::~basic_filebuf();

#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT filebuf::int_type filebuf::pbackfail(filebuf::int_type c);
	template _UCXXEXPORT filebuf * filebuf::open(const char* s, ios_base::openmode mode);
	template _UCXXEXPORT filebuf * filebuf::close();
	template _UCXXEXPORT filebuf::int_type filebuf::overflow(filebuf::int_type);
	template _UCXXEXPORT filebuf::int_type filebuf::underflow ();
	template _UCXXEXPORT streamsize filebuf::xsputn(const char* s, streamsize n);

	template _UCXXEXPORT basic_streambuf<char, char_traits<char> >*
		filebuf::setbuf(char * s, streamsize n);


#ifdef __UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__

	template _UCXXEXPORT basic_ofstream<char, char_traits<char> >::basic_ofstream();
	template _UCXXEXPORT basic_ofstream<char, char_traits<char> >::basic_ofstream(const char* s, ios_base::openmode mode);
	template _UCXXEXPORT basic_ofstream<char, char_traits<char> >::~basic_ofstream();

	template _UCXXEXPORT basic_ifstream<char, char_traits<char> >::basic_ifstream();
	template _UCXXEXPORT basic_ifstream<char, char_traits<char> >::basic_ifstream(const char* s, ios_base::openmode mode);
	template _UCXXEXPORT basic_ifstream<char, char_traits<char> >::~basic_ifstream();

#endif //__UCLIBCXX_EXPAND_CONSTRUCTORS_DESTRUCTORS__


#endif



#ifdef __UCLIBCXX_HAS_WCHAR__

template <> _UCXXEXPORT basic_filebuf<wchar_t, char_traits<wchar_t> >::int_type
	basic_filebuf<wchar_t, char_traits<wchar_t> >::overflow(int_type c)
{
	typedef basic_streambuf<wchar_t, char_traits<wchar_t> > wstreambuf;
	typedef char_traits<wchar_t> wtraits;

	if(is_open() == false){
		//Can't do much
		return wtraits::eof();
	}

	mbstate_t ps = { 0 };
	char out_array[8];
	size_t out_size;


	if( wstreambuf::pbase() != 0 ){

		//Write all possible character from the existing array first
		size_t chars_written = 0;
		while(wstreambuf::pbase() && (wstreambuf::pbase() + chars_written !=wstreambuf::pptr()) ){
			out_size = wcrtomb(out_array, wstreambuf::pbase()[chars_written], &ps);
			if(out_size == (size_t)(-1) || fwrite(out_array, out_size, 1, fp) == 0){
				break;
			}
			++chars_written;
		}

		if( wstreambuf::pbase() + chars_written == wstreambuf::pptr() ){
			wstreambuf::pbump(-chars_written);
		}else{
			//Shuffle data back into order
			size_t chars_left = wstreambuf::pptr() - wstreambuf::pbase() - chars_written;
			for(size_t i = 0; i < chars_left; ++i){
				wstreambuf::pbase()[i] = (wstreambuf::pptr() - chars_written)[i];
			}
			return wtraits::eof();
		}
	}

	if( !wtraits::eq_int_type(c, wtraits::eof()) ){
		out_size = wcrtomb(out_array, c, &ps);
		if(out_size == (size_t)(-1) || fwrite(out_array, out_size, 1, fp) == 0){
			return wtraits::eof();
		}
		return c;
	}

	return wtraits::not_eof(c);
}


template <> _UCXXEXPORT basic_filebuf<wchar_t, char_traits<wchar_t> >::int_type
        basic_filebuf<wchar_t, char_traits<wchar_t> >::underflow()
{
	/*Some variables used internally:
	Buffer pointers:

	charT * mgbeg;
	charT * mgnext;
	charT * mgend;

	eback() returns mgbeg
	gptr()  returns mgnext
	egptr() returns mgend

	gbump(int n) mgnext+=n
	*/

	typedef char_traits<wchar_t> traits;
	typedef basic_streambuf<wchar_t, traits> wstreambuf;


	if(wstreambuf::eback() == wstreambuf::gptr() && 0 != wstreambuf::eback()){	//Buffer is full
		return traits::to_int_type(*wstreambuf::gptr());
	}

        size_t in_size;

	wchar_t c = 0;
	wint_t wi = 0;
	in_size = 0;

	wi = fgetwc(fp);
	if(WEOF == wi){
		fprintf(stderr, "WEOF returned by fgetwc\n");
		return traits::eof();
	}

	c = traits::to_char_type(wi);

	if(wstreambuf::eback() == 0){
		return traits::to_int_type(c);
	}

	for(wchar_t * i = wstreambuf::gptr(); i < wstreambuf::egptr(); ++i){
		*(i-1) = *i;
	}

	*(wstreambuf::egptr()-1) = c;
	
	wstreambuf::mgnext -= 1;

	return traits::to_int_type(*wstreambuf::gptr());
}

#endif // __UCLIBCXX_HAS_WCHAR__


}
