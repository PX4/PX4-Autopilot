/*
 * gps.h
 *
 *  Created on: Mar 8, 2012
 *      Author: thomasgubler
 */

#ifndef GPS_H_
#define GPS_H

#include <stdbool.h>

struct arg_struct {
    int *fd_ptr;
    bool *thread_should_exit_ptr;
};

#endif /* GPS_H_ */
