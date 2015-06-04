#include <stdio.h>
#include <dlfcn.h>

#define STACK_SIZE 0x8000
static char __attribute__ ((aligned (16))) stack1[STACK_SIZE];

int main(int argc, char* argv[])
{
	int ret = 0;
	char *builtin[]={"libgcc.so", "libc.so", "libstdc++.so"};
	void *handle;
	char *error;
	void (*entry_function)() = NULL;

	printf("In DSPAL main\n");
	dlinit(3, builtin);
#if 0
	handle = dlopen ("libdspal_client.so", RTLD_LAZY);
	if (!handle) {
		printf("Error opening libdspal_client.so\n");
		return 1;
	}
	entry_function = dlsym(handle, "dspal_entry");
	if (((error = dlerror()) != NULL) || (entry_function == NULL)) {
		printf("Error dlsym for dspal_entry");
		ret = 2;
	}
	dlclose(handle);
#endif
	return ret;
}

