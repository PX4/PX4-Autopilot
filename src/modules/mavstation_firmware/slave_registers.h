
#ifndef __MAVSTATION_FIRMWARE_SLAVE_REGISTERS_H__
#define __MAVSTATION_FIRMWARE_SLAVE_REGISTERS_H__

#include <stdint.h>

#include "protocol.h"

/**
 * Register space
 */
void slave_registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);
int  slave_registers_get(uint8_t page, uint8_t offset, volatile uint16_t **values, unsigned *num_values);

/**
 * Register element accessors
 */
uint8_t slave_registers_get_debug_level(void);
uint8_t slave_registers_get_status_flags(void);
uint8_t slave_registers_get_setup_features(void);

#endif // __MAVSTATION_FIRMWARE_SLAVE_REGISTERS_H__
