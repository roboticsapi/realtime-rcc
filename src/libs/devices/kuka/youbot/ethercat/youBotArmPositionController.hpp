/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */


#ifndef YOUBOTARMPOSITIONCONTROLLER_HPP_
#define YOUBOTARMPOSITIONCONTROLLER_HPP_

#include "../device/youBotArmController.hpp"
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>

namespace kuka_youbot
{
	class youBotArmPositionController: public youBotArmController
	{
	public:
		youBotArmPositionController(youBotArmDataSource* dataSource, youBotArmControllable* controllable);
		virtual ~youBotArmPositionController();
		virtual void updateHook();

		// set gain of each joint
		void setP(int joint, float p);

	private:

		// get data+controll access
		youBotArmDataSource *m_dataSource;
		youBotArmControllable *m_controllable;

		// gain
		float P[5];
	};
}

#endif /* YOUBOTARMPOSITIONCONTROLLER_HPP_ */
