/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdint>

namespace board
{

void init();

__attribute__((noreturn))
void die(int error);

void setLed(bool state);

void restart();

constexpr unsigned UniqueIDSize = 12;

void readUniqueID(std::uint8_t bytes[UniqueIDSize]);

}
