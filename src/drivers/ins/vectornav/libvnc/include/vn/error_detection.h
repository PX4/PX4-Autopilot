#ifndef VNERRDET_H_INCLUDED
#define VNERRDET_H_INCLUDED

/** \brief Error-detection capabilities. */

#include "vn/enum.h"
#include "vn/int.h"
#include "vn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Computes the 8-bit XOR checksum of the provided data.
 *
 * \param[in] data Pointer to the start of data to perform the checksum of.
 * \param[in] length The number of bytes to include in the checksum.
 * \return The computed checksum.
 */
uint8_t VnChecksum8_compute(char const *data, size_t length);

/** \brief Computes the 16-bit CRC16-CCITT of the provided data.
 *
 * \param[in] data Pointer to the start of data to perform the CRC of.
 * \param[in] length The number of bytes to include in the CRC.
 * \return The computed CRC.
 */
uint16_t VnCrc16_compute(char const *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
