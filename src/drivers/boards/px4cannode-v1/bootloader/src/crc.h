#pragma once

#define CRC16_INITIAL 0xFFFFu
#define CRC16_OUTPUT_XOR 0x0000u
uint16_t crc16_add(uint16_t crc, uint8_t value);
uint16_t crc16_signature(uint16_t initial, size_t length,
                         const uint8_t *bytes);

#define CRC64_INITIAL 0xFFFFFFFFFFFFFFFFull
#define CRC64_OUTPUT_XOR 0xFFFFFFFFFFFFFFFFull
uint64_t crc64_add(uint64_t crc, uint8_t value);
