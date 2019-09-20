/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef YOUBOTGRIPPER_HPP_
#define YOUBOTGRIPPER_HPP_

#include <rcc/Device.hpp>
#include <rcc/DeviceFactory.hpp>

namespace kuka_youbot
{

	class RTT_EXPORT youBotGripper
	{
	public:

		virtual ~youBotGripper()
		{
		}

		virtual float getGripperPosition() = 0;
		virtual bool setGripperPosition(float value) = 0;
		virtual bool gripperBusy() = 0;

	};

}
#endif /* YOUBOTGRIPPER_HPP_ */
