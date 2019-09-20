/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotArmPositionController.hpp"
#include <rtt/Logger.hpp>

// FIXME: use real cycle time
#define YB_ARM_CYCLE_TIME 0.002

namespace kuka_youbot
{
	youBotArmPositionController::youBotArmPositionController(youBotArmDataSource* dataSource,
			youBotArmControllable* controllable)
	{
		m_dataSource = dataSource;
		m_controllable = controllable;

		// default values
		P[0] = 0.013;
		P[1] = 0.03;
		P[2] = 0.04;
		P[3] = 0.02;
		P[4] = 0.013;
	}

	youBotArmPositionController::~youBotArmPositionController()
	{
		// TODO Auto-generated destructor stub
	}

	void youBotArmPositionController::updateHook()
	{
		// set the commanded velocity to each joint
		for (int i = 0; i < 5; i++)
		{
			// err = (msrPos[i] - cmdPos[i]) / CYCLE_TIME;
			double err = (m_dataSource->getMeasuredJointPosition(i) -
					m_dataSource->getCommandedJointPosition(i)) / m_dataSource->getCycleTime();

			if(fabs(m_dataSource->getMeasuredJointPosition(i) -
					m_dataSource->getCommandedJointPosition(i)) > 0.5) {
				RTT::log(RTT::Warning) << "Axis #" << i << " tried to jump from " <<
						m_dataSource->getMeasuredJointPosition(i) << " to " <<
						m_dataSource->getCommandedJointPosition(i) << "." << RTT::endlog();
				m_controllable->setVelocity(i,0);
				continue;
			}

			double vel = m_dataSource->getCommandedJointVelocity(i);
			double speed = vel - err * P[i];
			m_controllable->setVelocity(i,speed);
		}
	}

	// set gain to given joint
	void youBotArmPositionController::setP(int joint, float p)
	{
		// security check
		if (joint<0 || joint> 4) return;

		// set gain
		P[joint] = p;
	}
} // end of namespace yb

