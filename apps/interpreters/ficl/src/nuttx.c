#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "ficl.h"

void *ficlMalloc(size_t size)
{
  return malloc(size);
}

void *ficlRealloc(void *p, size_t size)
{
  return realloc(p, size);
}

void ficlFree(void *p)
{
  free(p);
}

void  ficlCallbackDefaultTextOut(ficlCallback *callback, char *message)
{
  FICL_IGNORE(callback);
  if (message != NULL)
      fputs(message, stdout);
  else
      fflush(stdout);
  return;
}

int ficlFileStatus(char *filename, int *status)
{
    struct stat statbuf;
    if (stat(filename, &statbuf) == 0)
    {
        *status = statbuf.st_mode;
        return 0;
    }
    *status = ENOENT;
    return -1;
}

long ficlFileSize(ficlFile *ff)
{
    struct stat statbuf;
    if (ff == NULL)
        return -1;
	
    statbuf.st_size = -1;
    if (fstat(fileno(ff->f), &statbuf) != 0)
        return -1;
	
    return statbuf.st_size;
}

void ficlSystemCompilePlatform(ficlSystem *system)
{
    return;
}


