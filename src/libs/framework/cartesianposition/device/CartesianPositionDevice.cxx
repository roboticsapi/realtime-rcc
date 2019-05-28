/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "CartesianPositionDevice.hpp"
#include <kdl/frames.hpp>

namespace cartesianposition
{

	CartesianPositionDevice::CartesianPositionDevice():
			cmdTime(-1), cmdPos(), cmdVel()
	{
		dumpRPIPos = new RPI::RoundRobinLog<double>[3];

		dumpRPIPos[0].setName("X|Position|RPI");
		dumpRPIPos[1].setName("Y|Position|RPI");
		dumpRPIPos[2].setName("Yaw|Position|RPI");
	}

	CartesianPositionDevice::~CartesianPositionDevice()
	{
		delete[] dumpRPIPos; dumpRPIPos = 0;
	}

	std::list<RPI::CrashDumper*> CartesianPositionDevice::getCrashDumpers() const
	{
		std::list<RPI::CrashDumper*> ret;
		for(int i=0; i<3; i++) {
			ret.push_back(&dumpRPIPos[i]);
		}
		return ret;
	}


	void CartesianPositionDevice::setPosition(KDL::Frame position, RTT::os::TimeService::nsecs time)
	{
		if(position.p.x() != position.p.x() ||
				position.p.y() != position.p.y() ||
				position.M.GetRot().z() != position.M.GetRot().z()) {
			return;
		}
		if(cmdTime != -1) {
			cmdVel = position.M.Inverse() * KDL::diff(cmdPos, position, (time - cmdTime) / 1e9);
		}
		cmdPos = position;
		cmdTime = time;
		dumpRPIPos[0].put(position.p.x(), time);
		dumpRPIPos[1].put(position.p.y(), time);
		dumpRPIPos[2].put(position.M.GetRot().z(), time);
	}

	void CartesianPositionDevice::initPosition(KDL::Frame position, KDL::Twist velocity)
	{
		cmdPos = position;
		cmdVel = velocity;
		cmdTime = -1;
	}


	KDL::Frame CartesianPositionDevice::getCommandedPosition()
	{
		return cmdPos;
	}

	KDL::Twist CartesianPositionDevice::getCommandedVelocity()
	{
		return cmdVel;
	}

} /* namespace cartesianposition */
