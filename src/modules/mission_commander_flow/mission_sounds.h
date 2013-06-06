/*
 * sounds.h
 *
 *  Created on: Feb 26, 2013
 *      Author: samuezih
 */

#ifndef MISSION_SOUNDS_H_
#define MISSION_SOUNDS_H_

int mission_sounds_init(void);
void mission_sounds_deinit(void);
void tune_mission_started(void);
void tune_mission_aborted(void);
void tune_mission_accomplished(void);

#endif /* MISSION_SOUNDS_H_ */
