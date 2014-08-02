/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 */

#include <stdint.h>

namespace board
{

static const unsigned UniqueIDSize = 16;

void readUniqueID(uint8_t out_uid[UniqueIDSize]);

void setStatusLed(bool state);
void setErrorLed(bool state);

void resetWatchdog();

}
