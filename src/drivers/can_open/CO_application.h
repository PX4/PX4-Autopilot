/**
 * Application interface for CANopenNode.
 *
 * @file        CO_application.h
 * @ingroup     CO_applicationLinux
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CO_APPLICATION_H
#define CO_APPLICATION_H

#include "CANopen.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_applicationLinux Application interface to CANopenNode
 * Application interface, similar to Arduino, extended to CANopen and
 * additional, realtime thread.
 *
 * @ingroup CO_socketCAN
 * @{
 */

/**
 * Function is called once on the program startup, after Object dictionary
 * initialization and before CANopen initialization.
 *
 * @param [in,out] bitRate Unused in PX4 implementation. Use parameter system.
 * @param [in,out] nodeId Unused in PX4 implementation.  Use parameter system.
 * @param [out] errInfo Variable may indicate error information - index of
 * erroneous OD entry.
 *
 * @return @ref CO_ReturnError_t CO_ERROR_NO in case of success.
 */
CO_ReturnError_t app_programStart(uint16_t *bitRate,
				  uint8_t *nodeId,
				  uint32_t *errInfo);


/**
 * Function is called after CANopen communication reset.
 *
 * @param co CANopen object.
 */
void app_communicationReset(CO_t *co);


/**
 * Function is called just before program ends.
 */
void app_programEnd(void);


/**
 * Function is called every 5s
 *
 * @param co CANopen object.
 * @param timer1usDiff Time difference since last call in microseconds
 */
void app_programAsync(CO_t *co, uint32_t timer1usDiff);


/**
 * Function is called every 5ms by default.
 *
 * Code inside this function must be executed fast.
 *
 * @param co CANopen object.
 * @param timer1usDiff Time difference since last call in microseconds
 */
void app_programRt(CO_t *co, uint32_t timer1usDiff);

/** @} */ /* CO_applicationLinux */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_APPLICATION_H */
