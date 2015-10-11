/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 */

#include <cstdint>

namespace board
{

#if __GNUC__
__attribute__((noreturn))
#endif
void die();

static constexpr unsigned UniqueIDSize = 16;

void readUniqueID(std::uint8_t out_uid[UniqueIDSize]);

void setStatusLed(bool state);
void setErrorLed(bool state);

void resetWatchdog();

void syslog(const char* msg);

}
