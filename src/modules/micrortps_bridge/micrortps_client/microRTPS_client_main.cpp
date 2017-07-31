/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "microRTPS_transport.h"
#include "microRTPS_client.h"

#include <cinttypes>
#include <cstdio>
#include <ctime>
#include <termios.h>

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

extern "C" __EXPORT int micrortps_client_main(int argc, char *argv[]);

static int _rtps_task = -1;
bool _should_exit_task = false;
Transport_node *transport_node = nullptr;
struct options _options;

static void usage(const char *name)
{
    PX4_INFO("usage: %s start|stop [options]\n\n"
             "  -t <transport>          [UART|UDP] Default UART\n"
             "  -d <device>             UART device. Default /dev/ttyACM0\n"
             "  -u <update_time_ms>     Time in ms for uORB subscribed topics update. Default 0\n"
             "  -l <loops>              How many iterations will this program have. -1 for infinite. Default 10000\n"
             "  -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms\n"
             "  -b <baudrate>           UART device baudrate. Default 460800\n"
             "  -p <poll_ms>            Time in ms to poll over UART. Default 1ms\n"
             "  -r <reception port>     UDP port for receiving. Default 2019\n"
             "  -s <sending port>       UDP port for sending. Default 2020\n",
             name);
}

static int parse_options(int argc, char *argv[])
{
    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "t:d:u:l:w:b:p:r:s:", &myoptind, &myoptarg)) != EOF)
    {
        switch (ch)
        {
            case 't': _options.transport      = strcmp(myoptarg, "UDP") == 0?
                                                 options::eTransports::UDP
                                                :options::eTransports::UART;      break;
            case 'd': if (nullptr != myoptarg) strcpy(_options.device, myoptarg); break;
            case 'u': _options.update_time_ms = strtol(myoptarg, nullptr, 10);    break;
            case 'l': _options.loops          = strtol(myoptarg, nullptr, 10);    break;
            case 'w': _options.sleep_ms       = strtol(myoptarg, nullptr, 10);    break;
            case 'b': _options.baudrate       = strtoul(optarg, nullptr, 10);     break;
            case 'p': _options.poll_ms        = strtol(optarg, nullptr, 10);      break;
            case 'r': _options.recv_port      = strtoul(optarg, nullptr, 10);     break;
            case 's': _options.send_port      = strtoul(optarg, nullptr, 10);     break;
            default:
                usage(argv[1]);
            return -1;
        }
    }

    return 0;
}

static int micrortps_start(int argc, char *argv[])
{
    if (0 > parse_options(argc, argv))
    {
        printf("EXITING...\n");
        _rtps_task = -1;
        return -1;
    }

    switch (_options.transport)
    {
        case options::eTransports::UART:
        {
            transport_node = new UART_node(_options.device, _options.baudrate, _options.poll_ms);
            printf("\nUART transport: device: %s; baudrate: %d; sleep: %dms; poll: %dms\n\n",
                   _options.device, _options.baudrate, _options.sleep_ms, _options.poll_ms);
        }
        break;
        case options::eTransports::UDP:
        {
            transport_node = new UDP_node(_options.recv_port, _options.send_port);
            printf("\nUDP transport: recv port: %u; send port: %u; sleep: %dms\n\n",
                    _options.recv_port, _options.send_port, _options.sleep_ms);
        }
        break;
        default:
            _rtps_task = -1;
            printf("EXITING...\n");
        return -1;
    }

    if (0 > transport_node->init())
    {
        printf("EXITING...\n");
        _rtps_task = -1;
        return -1;
    }

    sleep(1);

    struct timespec begin;
    int total_read = 0;
    uint32_t received = 0;
    int loop = 0;
    micrortps_start_topics(begin, total_read, received, loop);

    struct timespec end;
    px4_clock_gettime(CLOCK_REALTIME, &end);
    double elapsed_secs = double(end.tv_sec - begin.tv_sec) + double(end.tv_nsec - begin.tv_nsec)/double(1000000000);
    printf("RECEIVED: %lu messages in %d LOOPS, %d bytes in %.03f seconds - %.02fKB/s\n\n",
            (unsigned long)received, loop, total_read, elapsed_secs, (double)total_read/(1000*elapsed_secs));

    delete transport_node;
    transport_node = nullptr;
    PX4_INFO("exiting");
    fflush(stdout);

    _rtps_task = -1;

    return 0;
}

int micrortps_client_main(int argc, char *argv[])
{
    if (argc < 2)
    {
        usage(argv[0]);
        return -1;
    }

    if (!strcmp(argv[1], "start"))
    {
        if (_rtps_task != -1)
        {
            PX4_INFO("Already running");
            return -1;
        }

        _rtps_task = px4_task_spawn_cmd("rtps",
                SCHED_DEFAULT,
                SCHED_PRIORITY_DEFAULT,
                4096,
                (px4_main_t) micrortps_start,
                (char *const *)argv);

        if (_rtps_task < 0)
        {
            PX4_WARN("Could not start task");
            _rtps_task = -1;
            return -1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop"))
    {
        if (_rtps_task == -1)
        {
            PX4_INFO("Not running");
            return -1;
        }

        _should_exit_task = true;
        if (nullptr != transport_node) transport_node->close();
        return 0;
    }

    usage(argv[0]);

    return -1;
}
