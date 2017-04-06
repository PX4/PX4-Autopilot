// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <errno.h>

#include "UART_node.h"

#define DEFAULT_UART "/dev/ttyACM0"

UART_node::UART_node(): m_uart_filestream(0)
{

}

UART_node::~UART_node()
{
    close_uart();
}

uint8_t UART_node::init_uart(const char * uart_name)
{

    uint32_t speed = 115200;
       // Open a serial port
    m_uart_filestream = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_uart_filestream < 0)
    {
        printf("failed to open port: %s\n", uart_name);
        return 1;
    }

    // Try to set baud rate
    struct termios uart_config;
    int termios_state;
    // Back up the original uart configuration to restore it after exit
    if ((termios_state = tcgetattr(m_uart_filestream, &uart_config)) < 0)
    {
        printf("ERR GET CONF %s: %d\n", uart_name, termios_state);
        close(m_uart_filestream);
        return -1;
    }

    // Clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    // USB serial is indicated by /dev/ttyACM0
    if (strcmp(uart_name, "/dev/ttyACM0") != 0 && strcmp(uart_name, "/dev/ttyACM1") != 0)
    {
        // Set baud rate
        if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
        {
            printf("ERR SET BAUD %s: %d\n", uart_name, termios_state);
            close(m_uart_filestream);
            return -1;
        }
    }

    if ((termios_state = tcsetattr(m_uart_filestream, TCSANOW, &uart_config)) < 0)
    {
        printf("ERR SET CONF %s\n", uart_name);
        close(m_uart_filestream);
        return -1;
    }
    return 0;
}


uint8_t UART_node::close_uart()
{
    printf("Close UART\n");
    close(m_uart_filestream);
    return 0;
}


uint8_t UART_node::readFromUART(char* topic_ID, char buffer[], char rx_buffer[])
{
    if (m_uart_filestream == -1) return 2;

    // Read up to max_size characters from the port if they are there
    
    // static char rx_buffer[1024];
    uint32_t pos_to_write = 0;
    char aux = 0;
    int rx_length = 0;

    // if (0 == pos_to_write) memset(rx_buffer, 0, max_size);

    while (pos_to_write <= max_size) // Not enough
    {
        if (1 != read(m_uart_filestream, (void*)&aux, 1))
        {
            // int errsv = errno;
            // printf("UART Fail %d %d\n", rx_length, errsv);
            // printf("%d %d %d %d %d %d %d \n", EAGAIN, EBADF, EFAULT, EINTR, EINVAL, EIO, EISDIR);
            return 0;
        }

        rx_buffer[pos_to_write++] = aux;
        if (pos_to_write < 3) continue;

        if (aux == '>')
        {
            // Beginning
            if (0 == strncmp(rx_buffer + (pos_to_write - 3), ">>>", 3))
            {
                // memset(rx_buffer, 0, 1024);
                rx_buffer[0] = rx_buffer[1] = rx_buffer[2] = '>';
                pos_to_write = 3;
            }
        }
        else if (aux == '<')
        {
            // Ending
            if (0 == strncmp(rx_buffer + (pos_to_write - 3), "<<<", 3) &&
                0 == strncmp(rx_buffer, ">>>", 3))
            {
                rx_length = pos_to_write - 7;
                break;
            }
        }
        if (pos_to_write > max_size)
        {
            pos_to_write = 0;
            return 0;
        }
    }

    // memset(buffer, 0, max_size);
    // Now rx_buffer is [<,<,<,topic_ID,payloadStart, ... ,payloadEnd,>,>,>]
    *topic_ID = rx_buffer[3];
    memmove(buffer, rx_buffer + 4, rx_length);

    return rx_length;
}


uint8_t UART_node::writeToUART(const char topic_ID, char buffer[], uint32_t length)
{
    if (m_uart_filestream == -1) return 2;

    dprintf(m_uart_filestream, ">>>");
    dprintf(m_uart_filestream, "%c", topic_ID);    // topic_ID
    write(m_uart_filestream, buffer, length);
    dprintf(m_uart_filestream, "<<<");

    return 0;
}
