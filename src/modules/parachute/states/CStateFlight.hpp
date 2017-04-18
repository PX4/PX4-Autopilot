/*
 * CStateFlight.hpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#ifndef SRC_MODULES_PARACHUTE_CSTATEFLIGHT_HPP_
#define SRC_MODULES_PARACHUTE_CSTATEFLIGHT_HPP_

#include "../CState.hpp"
#include "../CStateMachine.hpp"

class CStateFlight : public CState
{
public:
	CStateFlight(CStateMachine* pParent
#if MAVLINK_VERBOSE > 0
		, int iMavLinkFd
#endif
		);
	virtual ~CStateFlight();

	void init() override;
	void run() override;

	static const float m_scfLimVal;
};



#endif /* SRC_MODULES_PARACHUTE_CSTATEFLIGHT_HPP_ */
