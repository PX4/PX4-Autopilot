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

/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
uint16_t const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

UART_node::UART_node(): m_uart_filestream(0)
{

}

UART_node::~UART_node()
{
    close_uart();
}

uint8_t UART_node::init_uart(const char * uart_name, uint32_t baudrate)
{
    // Open a serial port
    m_uart_filestream = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (m_uart_filestream < 0)
    {
        printf("failed to open port: %s\n", uart_name);
        return 1;
    }

    // If using shared UART, no need to set it up
    if (baudrate == 0) {
        return 0;
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
        if (cfsetispeed(&uart_config, baudrate) < 0 || cfsetospeed(&uart_config, baudrate) < 0)
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
    bool flush = false;
    while (0 < read(m_uart_filestream, (void*)&aux, 64))
    {
        //printf("%s ", aux);
        flush = true;
        usleep(1000);
    }
    if (flush) printf("flush\n");

    return 0;
}


uint8_t UART_node::close_uart()
{
    printf("Close UART\n");
    close(m_uart_filestream);
    return 0;
}

int16_t UART_node::readFromUART(char* topic_ID, char out_buffer[], size_t buffer_len)
{
    if (-1 == m_uart_filestream || nullptr == out_buffer)
        return -1;

    // Read up to max_size characters from the port if they are there

    //uint32_t &pos_to_write = rx_buff_pos;

    int len = read(m_uart_filestream, (void*)(rx_buffer + rx_buff_pos), sizeof(rx_buffer) - rx_buff_pos);

    if (len <= 0)
    {
        int errsv = errno;
        //printf("UART Fail %d\n", errsv);
        // printf("%d %d %d %d %d %d %d \n", EAGAIN, EBADF, EFAULT, EINTR, EINVAL, EIO, EISDIR);
        if (errsv && EAGAIN != errsv)
        {
            printf("UART Fail %d\n", errsv);
        }
        return -1;
    }
    rx_buff_pos += len;

    //printf("read %d\n", len);

    /*printf(">>> |");
    for (int i = 0; i < rx_buff_pos + len; ++i)printf(" %hhu", rx_buffer[i]);
    printf("\n");*/

    // We read some
    if (rx_buff_pos < sizeof(struct Header))
        return 0; //but not enough

    uint32_t msg_start_pos = 0;
    for (msg_start_pos = 0; msg_start_pos <= rx_buff_pos - sizeof(struct Header); ++msg_start_pos)
    {
        if ('>' == rx_buffer[msg_start_pos] && memcmp(rx_buffer + msg_start_pos, ">>>", 3) == 0)
        {
            //printf("start found at %u\n", msg_start_pos);
            break;
        }
    }

    // Start not found
    if (msg_start_pos > rx_buff_pos - sizeof(struct Header))
    {
        //printf("start not found, pos %u\n", last_valid_pos);
        // All we've checked so far is garbage, drop it - but save unchecked bytes
        memmove(rx_buffer, rx_buffer + msg_start_pos, rx_buff_pos - msg_start_pos);
        rx_buff_pos = rx_buff_pos - msg_start_pos;
        return -1;
    }

    /*
     * [>,>,>,topic_ID,seq,payload_length,CRCHigh,CRCLow,payloadStart, ... ,payloadEnd]
     */

    struct Header *header = (struct Header *)&rx_buffer[msg_start_pos];

    uint16_t payload_len = ((uint16_t)header->payload_len_h << 8) | header->payload_len_l;

    //printf("payload_len %hhu\n", payload_len);

    // We do not have a complete message yet
    if(msg_start_pos + sizeof(struct Header) + payload_len > rx_buff_pos)
    {
        // If there's garbage at the beginning, drop it
        if (msg_start_pos > 0)
        {
            printf("                                 (↓ %u)\n", msg_start_pos);

            memmove(rx_buffer, rx_buffer + msg_start_pos, rx_buff_pos - msg_start_pos);
            rx_buff_pos -= msg_start_pos;
        }
        //printf("do not have a complete message, pos %u\n", rx_buff_pos);
        return 0;
    }

    // We have the whole message, but buffer won't fit it
    if (buffer_len < payload_len)
        return -EMSGSIZE;

    int ret;
    uint16_t read_crc = ((uint16_t)header->crc_h << 8) | header->crc_l;
    uint16_t calc_crc = crc16((uint8_t*)rx_buffer + msg_start_pos + sizeof(struct Header), payload_len);
    if (read_crc != calc_crc) {
        printf("BAD CRC %u != %u\n", read_crc, calc_crc);
        ret = -1;
    } else {
        //printf("GOOD CRC %u == %u\n", read_crc, calc_crc);
        // copy message to outbuffer and set other return values
        memmove(out_buffer, rx_buffer + msg_start_pos + sizeof(struct Header), payload_len);
        *topic_ID = header->topic_ID;
        ret = payload_len;
    }

    // discard message from rx_buffer
    rx_buff_pos -= sizeof(struct Header) + payload_len;
    memmove(rx_buffer, rx_buffer + msg_start_pos + sizeof(struct Header) + payload_len, rx_buff_pos);

    return ret;
}

int16_t UART_node::writeToUART(const char topic_ID, char buffer[], uint16_t length, uint8_t seq)
{
    static struct Header header {
        .marker = {'>','>','>'}
    };

    if (m_uart_filestream == -1) return 2;

    // [>,>,>,topic_ID,seq,payload_length,CRCHigh,CRCLow,payload_start, ... ,payload_end]

    uint16_t crc = crc16((uint8_t*)buffer, length);

    int ret = 0;
    header.topic_ID = topic_ID;
    header.seq = seq;
    header.payload_len_h = (length >> 8) & 0xff;
    header.payload_len_l = length & 0xff;
    header.crc_h = (crc >> 8) & 0xff;
    header.crc_l = crc & 0xff;
    ret = write(m_uart_filestream, &header, sizeof(header));
    if (ret != sizeof(header))
        goto err;

    ret = write(m_uart_filestream, buffer, length);
    if (ret != sizeof(header))
        goto err;

    /*printf(">>>%hhd %c %c|", topic_ID, seq, (char)length);
    for (int i = 0; i < length; ++i)printf(" %hhu", buffer[i]);
    printf("\n");*/

    return length;

err:
    int errsv = errno;
    if (ret == -1 )
    {
        printf("                               => Writing error '%d'\n", errsv);
    }
    else
    {
        printf("                               => Wrote '%d' != length(%u) error '%d'\n", ret, length, errsv);
    }
    return ret;
}


uint16_t UART_node::crc16_byte(uint16_t crc, const uint8_t data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

uint16_t UART_node::crc16(uint8_t const *buffer, size_t len)
{
	uint16_t crc = 0;
	while (len--) crc = crc16_byte(crc, *buffer++);
	return crc;
}
