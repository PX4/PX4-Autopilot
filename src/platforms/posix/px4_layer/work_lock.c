//#pragma once
#include <semaphore.h>
#include <stdio.h>
#include "work_lock.h"


extern sem_t _work_lock[];

void work_lock(int id)
{
  //printf("work_lock %d\n", id);
  sem_wait(&_work_lock[id]);
}

void work_unlock(int id)
{
  //printf("work_unlock %d\n", id);
  sem_post(&_work_lock[id]);
}
