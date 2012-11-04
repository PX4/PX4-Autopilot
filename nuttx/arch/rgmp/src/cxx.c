#include <rgmp/assert.h>
#include <rgmp/stdio.h>

int stderr = 2;

void __stack_chk_fail_local(void)
{
	panic("stack check fail\n");
}

int __sprintf_chk(char *str, int flag, size_t strlen, const char *format)
{
	return snprintf(str, strlen, format);
}

int dl_iterate_phdr(void* arg1, void* arg2)
{
    return -1;
}
