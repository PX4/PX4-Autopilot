/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_defines.h
 *
 * Generally used magic defines
 */

#pragma once

#if defined(__linux) || (defined(__APPLE__) && defined(__MACH__))
/*
 * Building for running within the ROS environment
 */
#define __EXPORT
// #define PX4_MAIN_FUNCTION(_prefix)
#define PX4_MAIN_FUNCTION(_prefix) int main(int argc, char **argv)
#define PX4_WARN ROS_WARN
#define PX4_INFO ROS_INFO
#define PX4_TOPIC(_name) #_name
#define PX4_TOPIC_T(_name) _name
#define PX4_SUBSCRIBE(_nodehandle, _name, _cbf) _nodehandle.subscribe(PX4_TOPIC(_name), _cbf);

#else
/*
 * Building for NuttX
 */

// #define PX4_MAIN_FUNCTION(_prefix) __EXPORT int _prefix##_main(int argc, char **argv)() { return main(argc, argv); }
// #define PX4_MAIN_FUNCTION(_prefix) extern "C" __EXPORT int _prefix##_main(int argc, char *argv[]) { return main(argc, argv); }
#define PX4_MAIN_FUNCTION(_prefix) extern "C" __EXPORT int _prefix##_main(int argc, char *argv[])
#define PX4_WARN warnx
#define PX4_WARN warnx
#define PX4_INFO warnx
#define PX4_TOPIC(_name) ORB_ID(_name)
#define PX4_TOPIC_T(_name) _name##_s
#define PX4_SUBSCRIBE(_nodehandle, _name, _cbf) _nodehandle.subscribe<PX4_TOPIC_T(_name)>(PX4_TOPIC(_name), 	[](const PX4_TOPIC_T(_name)& msg){ return _cbf(msg);})

#endif
