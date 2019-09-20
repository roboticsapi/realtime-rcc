/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotBase.hpp"

namespace kuka_youbot
{

	youBotBaseSimulation::youBotBaseSimulation(std::string name, RPI::parameter_t parameters):
		Device(name, parameters), RobotBaseDriverSimulation(), estop(false)
	{
		for(int i=0; i<4; i++)
			wheels[i] = wheelvel[i] = 0;
	}

	youBotBaseSimulation::~youBotBaseSimulation()
	{
	}

	youBotBaseSimulation* youBotBaseSimulation::createDevice(std::string name, RPI::parameter_t parameters)
	{
		youBotBaseSimulation* ret = new youBotBaseSimulation(name, parameters);
		ret->configure();
		ret->start();

		return ret;
	}


	std::set<std::string> youBotBaseSimulation::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void youBotBaseSimulation::updateParameters()
	{
	}

	void youBotBaseSimulation::setEStop(bool value)
	{
		estop = value;
	}

	RPI::DeviceState youBotBaseSimulation::getDeviceState() const
	{
		return estop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL;
	}

	int youBotBaseSimulation::getBaseError()
	{
		return estop ? 1 : 0;
	}

	int youBotBaseSimulation::checkBaseVelocity(KDL::Twist velocity)
	{
		return 0;
	}

	int youBotBaseSimulation::getWheelCount() const
	{
		return 4;
	}
	double youBotBaseSimulation::getWheelPosition(int wheel) const
	{
		if(wheel < 0 || wheel > 4) return 0;
		return wheels[wheel];
	}
	double youBotBaseSimulation::getWheelVelocity(int wheel) const
	{
		if(wheel < 0 || wheel > 4) return 0;
		return wheelvel[wheel];
	}

	void youBotBaseSimulation::updateHook()
	{
		RobotBaseDriverSimulation::updateHook();
		KDL::Twist vel = getCommandedBaseVelocity();
		float dt = RBDS_CYCLETIME;
		kin.velInvKin(vel.vel.x(), vel.vel.y(), vel.rot.z(), wheelvel[0], wheelvel[1], wheelvel[2], wheelvel[3]);
		for(int i=0; i<4; i++) {
			wheels[i] += wheelvel[i] * dt;
		}
	}


	std::string youBotBaseSimulation::getRobotBaseResourceName()
	{
		return Device::getName();
	}

	std::string youBotBaseSimulation::getCartesianPositionResourceName()
	{
		return getRobotBaseResourceName();
	}

	int youBotBaseSimulation::getCartesianPositionDeviceError()
	{
		return getBaseError();
	}

}
