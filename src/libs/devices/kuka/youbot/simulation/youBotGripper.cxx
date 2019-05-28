/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotGripper.hpp"

namespace kuka_youbot
{

	youBotGripperSimulation::youBotGripperSimulation(std::string name, RPI::parameter_t parameters) :
			RPI::Device(name, parameters), isEStop(false),
			maxGripperDistance(0.023), gripperOffset(0), gripperPosition(0)
	{

	}
	youBotGripperSimulation::~youBotGripperSimulation()
	{

	}

	youBotGripperSimulation* youBotGripperSimulation::createDevice(std::string name, RPI::parameter_t parameters)
	{
		youBotGripperSimulation* ret = new youBotGripperSimulation(name, parameters);
		return ret;
	}

	void youBotGripperSimulation::setEStop(bool estop)
	{
		isEStop = estop;
	}

	void youBotGripperSimulation::updateParameters()
	{

	}

	std::set<std::string> youBotGripperSimulation::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	RPI::DeviceState youBotGripperSimulation::getDeviceState() const
	{
		return isEStop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL;
	}

	bool youBotGripperSimulation::setGripperPosition(float value)
	{
		bool success = checkGripperPositionInRange(value);
		if(success) gripperPosition = value;
		return success;
	}

	float youBotGripperSimulation::getGripperPosition()
	{
		return gripperPosition;
	}

	bool youBotGripperSimulation::gripperBusy()
	{
		return false;
	}

	bool youBotGripperSimulation::checkGripperPositionInRange(const float newPosition) const
	{
		float newPos_roundedUp = (ceilf(newPosition * 1000) / 1000);
		float upperRange = (ceilf((maxGripperDistance + gripperOffset) * 1000) / 1000);

		if (newPosition < 0 || newPos_roundedUp > upperRange)
		{
			return false;
		}
		return true;
	}

}

