#include <stdlib.h>
#include <new>

/*
  These are the heap access function exported by the SLPI DSP image.
*/
extern "C" {
	void *fc_heap_alloc(size_t size);
	void fc_heap_free(void *ptr);
}

/*
  Globally override new and delete so that it can use the correct
  heap manager for the Qurt platform.

  Note that new comes in multiple different variants. When new is used
  without std::nothrow the compiler is free to assume it will not fail
  as it assumes exceptions are enabled. This makes code like this
  unsafe when using -fno-exceptions:

    a = new b;
    if (a == nullptr) {
      handle_error()
    }

  The compiler may remove the error handling. With g++ you can use
  -fcheck-new to avoid this, but on clang++ the compiler accepts
  -fcheck-new as a valid flag, but doesn't implement it, and may remove
  the error checking. That makes using clang++ unsafe with
  -fno-exceptions if you ever call new without std::nothrow.
..It has been verified that hexagon-clang++ will remove the nullptr checks
  after new if any optimization is selected for the compiler.
*/

/*
  variant for new(std::nothrow), which is all that should be used in
  PX4
 */
void *operator new (size_t size, std::nothrow_t const &nothrow) noexcept
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}

void *operator new[](size_t size, std::nothrow_t const &nothrow) noexcept
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}

/*
  These variants are for new without std::nothrow. We don't want to ever
  use these from PX4 code
 */
void *operator new (size_t size)
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}


void *operator new[](size_t size)
{
	if (size < 1) {
		size = 1;
	}

	return (fc_heap_alloc(size));
}

/*
	Override delete to free up memory to correct heap
*/

void operator delete (void *p) noexcept
{
	if (p) { fc_heap_free(p); }
}

void operator delete[](void *ptr) noexcept
{
	if (ptr) { fc_heap_free(ptr); }
}
