/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTARMCONTROLLER_HPP_
#define YOUBOTARMCONTROLLER_HPP_

#include <rcc/Device.hpp>
#include <rcc/DeviceFactory.hpp>

namespace kuka_youbot
{

	class RTT_EXPORT youBotArmController {
	public:
		virtual ~youBotArmController() {}
		virtual void updateHook() = 0;
	};

	class RTT_EXPORT youBotArmControllable {
	public:
		virtual ~youBotArmControllable() {}
		virtual void setTorque(int joint, double torque) = 0;
		virtual void setVelocity(int joint, double velocity) = 0;
	};

	class RTT_EXPORT youBotArmDataSource {
	public:
		virtual ~youBotArmDataSource() {}
		virtual double getCycleTime() = 0;
		virtual double getCommandedJointPosition(int joint) = 0;
		virtual double getMeasuredJointPosition(int joint) = 0;
		virtual double getCommandedJointVelocity(int joint) = 0;
		virtual double getMeasuredJointVelocity(int joint) = 0;
		virtual double getCommandedJointAcceleration(int joint) = 0;
		virtual double getMeasuredJointAcceleration(int joint) = 0;
	};

}


#endif /* YOUBOTARMCONTROLLER_HPP_ */
