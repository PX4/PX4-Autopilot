// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#pragma once

#include <string>
#include <cstring>
#include <arpa/inet.h>


class Transport_node
{
public:
    Transport_node();
    virtual ~Transport_node();

    virtual int init() {return 0;}
    virtual uint8_t close() {return 0;}
    ssize_t read(char* topic_ID, char out_buffer[], size_t buffer_len);
    ssize_t write(const char topic_ID, char buffer[], size_t length);

protected:
    virtual ssize_t node_read(void *buffer, size_t len) = 0;
    virtual ssize_t node_write(void *buffer, size_t len) = 0;
    virtual bool fds_OK() = 0;
    uint16_t crc16_byte(uint16_t crc, const uint8_t data);
    uint16_t crc16(uint8_t const *buffer, size_t len);

protected:
    uint32_t rx_buff_pos;
    char rx_buffer[1024] = {};

private:
    struct __attribute__((packed)) Header
    {
        char marker[3];
        uint8_t topic_ID;
        uint8_t seq;
        uint8_t payload_len_h;
        uint8_t payload_len_l;
        uint8_t crc_h;
        uint8_t crc_l;
    };
};

class UART_node: public Transport_node
{
public:
    UART_node(const char *uart_name, uint32_t baudrate);
    virtual ~UART_node();

    int init();
    uint8_t close();

protected:
    ssize_t node_read(void *buffer, size_t len);
    ssize_t node_write(void *buffer, size_t len);
    bool fds_OK();

    int uart_fd;
    std::string uart_name;
    uint32_t baudrate;
};

class UDP_node: public Transport_node
{
public:
    UDP_node(uint16_t udp_port_recv, uint16_t udp_port_send);
    virtual ~UDP_node();

    int init();
    uint8_t close();

protected:
    int init_receiver(uint16_t udp_port);
    int init_sender(uint16_t udp_port);
    ssize_t node_read(void *buffer, size_t len);
    ssize_t node_write(void *buffer, size_t len);
    bool fds_OK();

    int sender_fd;
    int receiver_fd;
    uint16_t udp_port_recv;
    uint16_t udp_port_send;
    struct sockaddr_in sender_outaddr;
    struct sockaddr_in receiver_inaddr;
    struct sockaddr_in receiver_outaddr;
};
