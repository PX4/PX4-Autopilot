#include "crc_helper.h"

// Compute CRC word for a byte string.
uint16_t MakeCrc(const uint8_t *data, int count) {

  uint16_t crc = 0xffff;

  uint16_t i;
  for(i = 0; i < count; i++) {
    crc = ByteUpdateCrc(crc, data[i]);
  }
  return crc;
}

// Update a CRC accumulation with one data byte.
uint16_t ByteUpdateCrc(uint16_t crc, uint8_t data) {

  uint16_t x = (crc >> 8) ^ data;
  x ^= x >> 4;

  crc = (crc << 8) ^ (x << 12) ^ (x <<5) ^ x;
  return crc;
}

// Update a CRC accumulation with several data bytes.
uint16_t ArrayUpdateCrc(uint16_t crc, const uint8_t *data, int count) {

  uint16_t i;
  for(i = 0; i < count; i++) {
    crc = ByteUpdateCrc(crc, data[i]);
  }
  return crc;
}
