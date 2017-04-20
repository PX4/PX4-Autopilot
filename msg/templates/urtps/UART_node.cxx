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
    m_uart_filestream = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

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
        return 1;
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
            return 1;
        }
    }

    if ((termios_state = tcsetattr(m_uart_filestream, TCSANOW, &uart_config)) < 0)
    {
        printf("ERR SET CONF %s\n", uart_name);
        close(m_uart_filestream);
        return 1;
    }

    char aux[64];
    while (0 < read(m_uart_filestream, (void*)&aux, 64))
    {
        //printf("%s ", aux);
        usleep(1000);
    }
    printf("flush\n");

    return 0;
}


uint8_t UART_node::close_uart()
{
    printf("Close UART\n");
    close(m_uart_filestream);
    return 0;
}


uint8_t UART_node::readFromUART(char* topic_ID, char out_buffer[], char rx_buffer[], uint32_t &rx_buff_pos)
{
    if (-1 == m_uart_filestream ||
        nullptr == out_buffer ||
        nullptr == rx_buffer)
        return 2;

    // Read up to max_size characters from the port if they are there
    
    //uint32_t &pos_to_write = rx_buff_pos;
    char aux = 0;
    int rx_length = 0;

    while (rx_buff_pos <= max_size) // Not enough
    {
        if (1 != read(m_uart_filestream, (void*)&aux, 1))
        {
            //int errsv = errno;
            //printf("UART Fail %d\n", errsv);
            // printf("%d %d %d %d %d %d %d \n", EAGAIN, EBADF, EFAULT, EINTR, EINVAL, EIO, EISDIR);
            //if (errsv && EAGAIN != errsv) printf("UART Fail %d\n", errsv);
            return 0;
        }

        rx_buffer[rx_buff_pos++] = aux;
        if (rx_buff_pos < 3) continue;

        if (aux == '>')
        {
            // Beginning
            if (0 == strncmp(rx_buffer + (rx_buff_pos - 3), ">>>", 3))
            {
                if (rx_buff_pos > 3)
                {
                    printf("æ");
                    //printf("# %u bytes lost 1\n", rx_buff_pos - 3 + 1);
                    rx_buffer[0] = rx_buffer[1] = rx_buffer[2] = '>';
                    rx_buff_pos = 3;
                }
            }
        }
        else if (aux == '<')
        {
            // Ending
            if (0 == strncmp(rx_buffer + (rx_buff_pos - 3), "<<<", 3) &&
                0 == strncmp(rx_buffer, ">>>", 3))
            {
                rx_length = rx_buff_pos - 7;
                printf("# (@,%d) ", rx_length);
                break;
            }
        }
        if (rx_buff_pos > max_size)
        {
            printf("ß");
            // printf("# %u bytes lost 2\n", rx_buff_pos);
            rx_buff_pos = 0;
            return 0;
        }
    }

    // Now rx_buffer is [>,>,>,topic_ID,payloadStart, ... ,payloadEnd,<,<,<]
    *topic_ID = rx_buffer[3];
    memmove(out_buffer, rx_buffer + 4, rx_length);
    rx_buff_pos = 0;

    return rx_length;
}


uint8_t UART_node::writeToUART(const char topic_ID, char buffer[], uint32_t length)
{
    if (m_uart_filestream == -1) return 2;

    dprintf(m_uart_filestream, ">>>%c", topic_ID);    // topic_ID
    write(m_uart_filestream, buffer, length);
    dprintf(m_uart_filestream, "<<<");
    /*printf(">>>%hhd|", topic_ID);
    for (int i = 0; i < length; ++i)printf(" %hhu", buffer[i]);
    printf("<<<\n");*/


    return 0;
}
