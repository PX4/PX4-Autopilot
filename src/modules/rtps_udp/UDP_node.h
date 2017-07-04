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

class UDP_node
{
public:
    UDP_node();
    virtual ~UDP_node();

    int init(uint16_t udp_port_recv, uint16_t udp_port_send);
    int init_receiver(uint16_t udp_port);
    int init_sender(uint16_t udp_port);
    uint8_t close();
    int16_t read(char* topic_ID, char out_buffer[]);
    int16_t write(const char topic_ID, char buffer[], uint32_t length);

protected:
    uint16_t crc16_byte(uint16_t crc, const uint8_t data);
    uint16_t crc16(uint8_t const *buffer, size_t len);

protected:

    int sender_fd;
    int receiver_fd;
    struct sockaddr_in sender_outaddr;
    struct sockaddr_in receiver_inaddr;
    struct sockaddr_in receiver_outaddr;
    uint32_t rx_buff_pos = 0;
    char rx_buffer[1024] = {};

private:
    struct __attribute__((packed)) Header {
        char marker[3];
        uint8_t topic_ID;
        uint8_t seq;
        uint8_t payload_len_h;
        uint8_t payload_len_l;
        uint8_t crc_h;
        uint8_t crc_l;
    };
};
