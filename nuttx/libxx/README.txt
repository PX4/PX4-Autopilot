libxx/README.txt
^^^^^^^^^^^^^^^^

This directory contains a fragmentary C++ library that will allow
to build only the simplest of C++ applications.  In the deeply
embedded world, that is probably all that is necessary.  If you
have a need for more extensive C++ support, the following libraries
are recommended:

 - libstdc++	(part of GCC)
 - STLport	http://www.stlport.org/ 
 - uClibc++	http://cxx.uclibc.org/

At present, only the following are supported here:

 - void *operator new(std::size_t nbytes);
 - void operator delete(void* ptr);