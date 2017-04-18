/*
 * CStateLanding.hpp
 *
 *  Created on: Aug 28, 2015
 *      Author: friedrich
 */

#ifndef SRC_MODULES_PARACHUTE_CSTATELANDING_HPP_
#define SRC_MODULES_PARACHUTE_CSTATELANDING_HPP_

#include "../CState.hpp"
#include "../CStateMachine.hpp"


class CStateLanding : public CState
{
public:
	CStateLanding(CStateMachine* pParent
#if MAVLINK_VERBOSE > 0
		, int iMavLinkFd
#endif
		);
	virtual ~CStateLanding();

	void init() override;
	void run() override;
};



#endif /* SRC_MODULES_PARACHUTE_CSTATELANDING_HPP_ */
