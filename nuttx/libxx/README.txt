libxx/README.txt
^^^^^^^^^^^^^^^^

This directory contains a fragmentary C++ library that will allow
to build only the simplest of C++ applications.  In the deeply
embedded world, that is probably all that is necessary.  If you
have a need for more extensive C++ support, the following libraries
are recommended:

 - libstdc++  (part of GCC)
 - STLport    http://www.stlport.org/ 
 - uClibc++   http://cxx.uclibc.org/
 - uSTL       http://ustl.sourceforge.net/

There is a version of uClibc++ that is customized for NuttX that can
be found here:  misc/uClibc++.  See misc/uClibc++ for installation
instructions.

At present, only the following are supported here:

 - void *operator new(std::size_t nbytes);
 - void operator delete(void* ptr);
 - void operator delete[](void *ptr);
 - void __cxa_pure_virtual(void);
 - int __aeabi_atexit(void* object, void (*destroyer)(void*), void *dso_handle);
 - int __cxa_atexit(__cxa_exitfunc_t func, FAR void *arg, FAR void *dso_handle);
 
operator new
------------

  This operator should take a type of size_t.  But size_t has an unknown underlying
  type.  In the nuttx sys/types.h header file, size_t is typed as uint32_t
  (which is determined by architecture-specific logic).  But the C++
  compiler may believe that size_t is of a different type resulting in
  compilation errors in the operator.  Using the underlying integer type
  instead of size_t seems to resolve the compilation issues. Need to
  REVISIT this.

  Once some C++ compilers, this will cause an error:

    Problem:     "'operator new' takes size_t ('...') as first parameter"
    Workaround:  Add -fpermissive to the compilation flags
