#pragma once

#include <stdbool.h>
#include <stdint.h>

void Crc8Init(const uint8_t poly);
uint8_t Crc8Calc(const uint8_t *data, uint8_t size);
