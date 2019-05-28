/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "RobotBaseSim.hpp"

namespace robotbase
{

	RobotBaseSim::RobotBaseSim(std::string name, RPI::parameter_t parameters):
		Device(name, parameters), RobotBaseDriverSimulation(), estop(false)
	{
	}

	RobotBaseSim::~RobotBaseSim()
	{
	}

	RobotBaseSim* RobotBaseSim::createDevice(std::string name, RPI::parameter_t parameters)
	{
		RobotBaseSim* ret = new RobotBaseSim(name, parameters);
		ret->configure();
		ret->start();

		return ret;
	}


	std::set<std::string> RobotBaseSim::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void RobotBaseSim::updateParameters()
	{
	}

	void RobotBaseSim::setEStop(bool value)
	{
		estop = value;
	}

	RPI::DeviceState RobotBaseSim::getDeviceState() const
	{
		return estop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL;
	}

	int RobotBaseSim::getBaseError()
	{
		return estop ? 1 : 0;
	}

	int RobotBaseSim::checkBaseVelocity(KDL::Twist velocity)
	{
		return 0;
	}

	std::string RobotBaseSim::getRobotBaseResourceName()
	{
		return Device::getName();
	}


	std::string RobotBaseSim::getCartesianPositionResourceName()
	{
		return getRobotBaseResourceName();
	}

	int RobotBaseSim::getCartesianPositionDeviceError()
	{
		return getBaseError();
	}



} /* namespace robotbase */
