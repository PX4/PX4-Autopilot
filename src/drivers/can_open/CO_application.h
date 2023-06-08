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
 * @param [in,out] bitRate Stored CAN bit rate, can be overridden.
 * @param [in,out] nodeId Stored CANopen NodeId, can be overridden.
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
 * Function is called cyclically from main().
 *
 * Place for the slower code (all must be non-blocking).
 *
 * @warning
 * Mind race conditions between this functions and app_programRt(), which
 * run from the realtime thread. If accessing Object dictionary variable which
 * is also mappable to PDO, it is necessary to use CO_LOCK_OD() and
 * CO_UNLOCK_OD() macros from @ref CO_critical_sections.
 *
 * @param co CANopen object.
 * @param timer1usDiff Time difference since last call in microseconds
 */
void app_programAsync(CO_t *co, uint32_t timer1usDiff);


/**
 * Function is called cyclically from realtime thread at constant intervals.
 *
 * Code inside this function must be executed fast. Take care on race conditions
 * with app_programAsync.
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
