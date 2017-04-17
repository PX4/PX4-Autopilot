/*!
 *	\file		sbgTime.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		02/04/07
 *
 *	\brief		Time functions for SBG Systems projects.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef __SBG_TIME_H__
#define __SBG_TIME_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Command operations                                                 -//
//----------------------------------------------------------------------//

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
uint32 sbgGetTime(void);

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
void sbgSleep(uint32 ms);

#endif
