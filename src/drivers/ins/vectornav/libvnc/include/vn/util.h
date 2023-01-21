#ifndef VNUTIL_H_INCLUDED
#define VNUTIL_H_INCLUDED

#include <stddef.h>

#include "vn/int.h"
#include "vn/bool.h"
#include "vn/error.h"
#include "vn/math/matrix.h"
#include "vn/math/vector.h"
#include "vn/util/export.h"
#include "vn/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Defines for the specific version of the VectorNav library. */
#define VNAPI_MAJOR		1
#define VNAPI_MINOR		2
#define VNAPI_PATCH		0
#define VNAPI_REVISION	126

/** \brief Returns the major version of the VectorNav library.
 *
 * \return The major version. */
int VnApi_major(void);

/** \brief Returns the minor version of the VectorNav library.
*
* \return The minor version. */
int VnApi_minor(void);

/** \brief Returns the patch version of the VectorNav library.
*
* \return The patch version. */
int VnApi_patch(void);

/** \brief Returns the revision version of the VectorNav library.
*
* \return The revision version. */
int VnApi_revision(void);

/** \brief Returns the full version of the VectorNav library as a string.
 *
 * \param[out] out The buffer to place the string.
 * \param[in] outLength The number of characters available in the out buffer.
 * \return Any errors encountered. */
VnError VnApi_getVersion(char *out, size_t outLength);

/** \brief Converts the single hexadecimal character to a uint8_t.
*
* \param[in] c The hexadecimal character to convert.
* \return The converted value.
*/
uint8_t VnUtil_toUint8FromHexChar(char c);

/** \brief Converts a 2 character hexadecimal string to a uint8_t.
*
* \param[in] str Pointer to 2 characters representing a hexadecimal
*     number.
* \return The converted value.
*/
uint8_t VnUtil_toUint8FromHexStr(char const *str);

/** \brief Converts a 4 character hexadecimal string to a uint16_t.
*
* \param[in] str Pointer to 4 characters representing a hexadecimal
*     number.
* \return The converted value.
*/
uint16_t VnUtil_toUint16FromHexStr(char const *str);

/** \brief Converts a uint8_t to a hexadecimal string.
*
* \param[in] toConvert The uint8_t to convert.
* \param[in] output Pointer to the char array to write the converted value.
*     This will take 2 bytes for the output.
*/
void VnUtil_toHexStrFromUint8(uint8_t toConvert, char *output);

/** \brief Converts a uint16_t to a hexadecimal string.
*
* \param[in] toConvert The uint16_t to convert.
* \param[in] output Pointer to the char array to write the converted value.
*     This will take 4 bytes for the output.
*/
void VnUtil_toHexStrFromUint16(uint16_t toConvert, char *output);

/** \brief Converts a uint16_t to a string.
*
* \param[in] toConvert The uint16_t to convert.
* \param[in] output Pointer to the char array to write the converted value.
* \return The number of characters that were written.
*/
size_t VnUtil_toStrFromUint16(uint16_t toConvert, char *output);

/** \brief Returns the number of bits set.
 *
 * \param[in] data The data value to count the number of bits set.
 * \return The number of bits set.
 */
uint8_t DllExport VnUtil_countSetBitsUint8(uint8_t data);

/** \brief Converts a boolean value into a string.
 *
 * \param[out] out The value converted to a string.
 * \param[in] val The value to convert.
 */
void strFromBool(char *out, bool val);

/** \defgroup byteOrderers Byte Ordering Functions
* \brief This group of functions are useful for ordering of bytes
* sent/received from VectorNav sensors.
*
* \{ */

/** \brief Converts a 16-bit integer in sensor order to host order.
*
* \param[in] sensorOrdered The 16-bit integer in sensor order.
* \return The value converted to host ordered. */
uint16_t stoh16(uint16_t sensorOrdered);

/** \brief Converts a 32-bit integer in sensor order to host order.
*
* \param[in] sensorOrdered The 32-bit integer in sensor order.
* \return The value converted to host ordered. */
uint32_t stoh32(uint32_t sensorOrdered);

/** \brief Converts a 64-bit integer in sensor order to host order.
*
* \param[in] sensorOrdered The 64-bit integer in sensor order.
* \return The value converted to host ordered. */
uint64_t stoh64(uint64_t sensorOrdered);

/** \brief Converts a 16-bit integer in host order to sensor order.
*
* \param[in] hostOrdered The 16-bit integer in host order.
* \return The value converted to sensor ordered. */
uint16_t htos16(uint16_t hostOrdered);

/** \brief Converts a 32-bit integer in host order to sensor order.
*
* \param[in] hostOrdered The 32-bit integer in host order.
* \return The value converted to sensor ordered. */
uint32_t htos32(uint32_t hostOrdered);

/** \brief Converts a 64-bit integer in host order to sensor order.
*
* \param[in] hostOrdered The 64-bit integer in host order.
* \return The value converted to sensor ordered. */
uint64_t htos64(uint64_t hostOrdered);

/** \brief Converts a 4-byte float in host order to sensor order.
*
* \param[in] hostOrdered The 4-byte float in host order.
* \return The value converted to sensor ordered. */
float htosf4(float hostOrdered);

/** \brief Converts an 8-byte float in host order to sensor order.
*
* \param[in] hostOrdered The 8-byte float in host order.
* \return The value converted to sensor ordered. */
double htosf8(double hostOrdered);

/** \} */

/** \defgroup sensorValueExtractors Sensor Value Extractors
* \brief This group of methods is useful for extracting data from binary
* data received from a VectorNav sensor either from a UART binary or a
* SPI packet. Any necessary byte ordering will be performed.
*
* \{ */

/** \brief Extracts a uint16_t with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
uint16_t VnUtil_extractUint16(const char* pos);

/** \brief Extracts a uint32_t with appropriate byte reordering from the binary
 *      array received from a VectorNav sensor either from the UART binary or
 *      SPI packet.
 *
 * \param[in] pos The current position to extract the value from.
 * \return The extracted value. */
uint32_t VnUtil_extractUint32(const char* pos);

/** \brief Extracts a <c>float</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
float VnUtil_extractFloat(const char* pos);

/** \brief Extracts a <c>double</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
double VnUtil_extractDouble(const char* pos);

/*#if THIS_SHOULD_BE_MOVED_TO_MATH_C_PACK*/

/** \brief Extracts a <c>vec3f</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
vec3f VnUtil_extractVec3f(const char* pos);

/** \brief Extracts a <c>vec4f</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
vec4f VnUtil_extractVec4f(const char* pos);

/** \brief Extracts a <c>vec3d</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
vec3d VnUtil_extractVec3d(const char* pos);

/** \brief Extracts a <c>mat3f</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
mat3f VnUtil_extractMat3f(const char* pos);

/** \brief Extracts a <c>GpsDop</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
GpsDop VnUtil_extractGpsDop(const char* pos);

/** \brief Extracts a <c>TimeUtc</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
TimeUtc VnUtil_extractTimeUtc(const char* pos);

/** \brief Extracts a <c>TimeInfo</c> with appropriate byte reordering from the binary
*      array received from a VectorNav sensor either from the UART binary or
*      SPI packet.
*
* \param[in] pos The current position to extract the value from.
* \return The extracted value. */
TimeInfo VnUtil_extractTimeInfo(const char* pos);

/*#endif*/

/* \} */

#ifdef __cplusplus
}
#endif

#endif
