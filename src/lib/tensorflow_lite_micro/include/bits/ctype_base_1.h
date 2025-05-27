// Locale support -*- C++ -*-

// Copyright (C) 2000-2023 Free Software Foundation, Inc.
//
// This file is part of the GNU ISO C++ Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the GCC Runtime Library Exception, version
// 3.1, as published by the Free Software Foundation.

// You should have received a copy of the GNU General Public License and
// a copy of the GCC Runtime Library Exception along with this program;
// see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
// <http://www.gnu.org/licenses/>.

//
// ISO C++ 14882: 22.1  Locales
//

// Information as gleaned from /usr/include/ctype.h

namespace std _GLIBCXX_VISIBILITY(default)
{
_GLIBCXX_BEGIN_NAMESPACE_VERSION

/// @brief  Base class for ctype.
struct ctype_base {
	// Non-standard typedefs.
	typedef const int *__to_type;

	// NB: Offsets into ctype<char>::_M_table force a particular size
	// on the mask type. Because of this, we don't use an enum.
	typedef char 		mask;
	// Define the character classification masks
	static const mask upper     = 0x01; // _U
	static const mask lower 	= 0x02; // _L
	static const mask alpha 	= 0x03; // _U | _L
	static const mask digit 	= 0x04; // _N
	static const mask xdigit 	= 0x08; // _X | _N
	static const mask space 	= 0x10; // _S
	static const mask print 	= 0x20; // _P | _U | _L | _N | _B
	static const mask graph 	= 0x40; // _P | _U | _L | _N
	static const mask cntrl 	= 0x80; // _C
	static const mask punct 	= 0x100; // _P
	static const mask alnum 	= 0x200; // _U | _L | _N
#if __cplusplus >= 201103L
	static const mask blank 	= 0x400; // space
#endif
};

_GLIBCXX_END_NAMESPACE_VERSION
} // namespace
