#ifndef CRC_HELPER_H
#define	CRC_HELPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Compute CRC word for a byte string.
 */
uint16_t MakeCrc(const uint8_t *data, int count);

/* Update a CRC accumulation with one data byte.
 */
uint16_t ByteUpdateCrc(uint16_t crc, uint8_t data);

/* Update a CRC accumulation with several data bytes.
 */
uint16_t ArrayUpdateCrc(uint16_t crc, const uint8_t *data, int count);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif