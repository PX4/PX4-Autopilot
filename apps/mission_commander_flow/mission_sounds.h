/*
 * sounds.h
 *
 *  Created on: Feb 26, 2013
 *      Author: samuezih
 */

#ifndef SOUNDS_H_
#define SOUNDS_H_

int mission_sounds_init(void);
void mission_sounds_deinit(void);
void tune_mission_started(void);
void tune_mission_aborded(void);
void tune_mission_accomplished(void);

#endif /* SOUNDS_H_ */
