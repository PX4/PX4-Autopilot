#include <stdlib.h>
#include <new>
#include <px4_log.h>

/*
  These are the heap access function exported by the SLPI DSP image.
*/
extern "C" {
	void *fc_heap_alloc(size_t size);
	void fc_heap_free(void* ptr);
	size_t fc_heap_size(void);
    size_t fc_heap_usage(void);
}

/*
  Convenience class to collect and print heap statistics
*/
class getAllocStats {
public:
	void printStats() {
		PX4_INFO("new calls: %u, bytes: %u", _new, _new_bytes);
		PX4_INFO("new array calls: %u, bytes: %u", _new_array, _new_array_bytes);
		PX4_INFO("new nothrow calls: %u, bytes: %u", _new_nt, _new_nt_bytes);
		PX4_INFO("new nothrow array calls: %u, bytes: %u", _new_array_nt, _new_array_nt_bytes);
		PX4_INFO("delete calls: %u, bytes: %u", _delete, _delete_bytes);
		PX4_INFO("delete array calls: %u, bytes: %u", _delete_array, _delete_array_bytes);
		PX4_INFO("Total heap size: %u, currently using: %u", fc_heap_size(), fc_heap_usage());
	}

	uint32_t _new;
	uint32_t _new_bytes;
	uint32_t _new_array;
	uint32_t _new_array_bytes;
	uint32_t _new_nt;
	uint32_t _new_nt_bytes;
	uint32_t _new_array_nt;
	uint32_t _new_array_nt_bytes;
	uint32_t _delete;
	uint32_t _delete_bytes;
	uint32_t _delete_array;
	uint32_t _delete_array_bytes;

} allocStats;

extern "C" void printAllocStats(void);

void printAllocStats(void) {
	allocStats.printStats();
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
void * operator new(size_t size, std::nothrow_t const &nothrow) noexcept
{
	allocStats._new_nt++;
	allocStats._new_nt_bytes += size;
	
    if (size < 1) {
        size = 1;
    }
    return(fc_heap_alloc(size));
}

void * operator new[](size_t size, std::nothrow_t const &nothrow) noexcept
{
	allocStats._new_array_nt++;
	allocStats._new_array_nt_bytes += size;
	
    if (size < 1) {
        size = 1;
    }
    return(fc_heap_alloc(size));
}

/*
  These variants are for new without std::nothrow. We don't want to ever
  use these from PX4 code
 */
void * operator new(size_t size)
{
	allocStats._new++;
	allocStats._new_bytes += size;

    if (size < 1) {
        size = 1;
    }
    return(fc_heap_alloc(size));
}


void * operator new[](size_t size)
{
	allocStats._new_array++;
	allocStats._new_array_bytes += size;

    if (size < 1) {
        size = 1;
    }
    return(fc_heap_alloc(size));
}

/*
	Override delete to free up memory to correct heap
*/

void operator delete(void *p) noexcept
{
	allocStats._delete++;

    if (p) fc_heap_free(p);
}

void operator delete[](void * ptr) noexcept
{
	allocStats._delete_array++;

    if (ptr) fc_heap_free(ptr);
}
