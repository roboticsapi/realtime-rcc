/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "RobotBaseDriverSimulation.hpp"
#include <rtt/Activity.hpp>

namespace robotbase
{

	RobotBaseDriverSimulation::RobotBaseDriverSimulation() :
			TaskContext("RobotBaseSimulation"), vel(), lastVel(0)
	{
		this->setActivity(
				new RTT::Activity(ORO_SCHED_OTHER, RTT::os::LowestPriority, RBDS_CYCLETIME, 0, "RobotBaseSimulation"));
	}

	RobotBaseDriverSimulation::~RobotBaseDriverSimulation()
	{

	}

	void RobotBaseDriverSimulation::updateHook()
	{
		if(CartesianPositionDevice::cmdTime < lastVel) {
			if(RTT::os::TimeService::Instance()->getNSecs(lastVel) > 1e8)
			{
				vel = KDL::Twist();
			}
			KDL::Frame pos = getCommandedPosition();
			pos = KDL::addDelta(pos, pos.M.Inverse(vel), RBDS_CYCLETIME);
			initPosition(pos, vel);
		} else if(RTT::os::TimeService::Instance()->getNSecs(CartesianPositionDevice::cmdTime) > 1e8) {
			vel = KDL::Twist();
			KDL::Frame pos = getCommandedPosition();
			initPosition(pos, vel);
		}
	}

	void RobotBaseDriverSimulation::setBaseVelocity(KDL::Twist velocity)
	{
		if (checkBaseVelocity(velocity) == 0)
		{
			vel = velocity;
			lastVel = RTT::os::TimeService::Instance()->getNSecs();
		}
	}

	KDL::Frame RobotBaseDriverSimulation::getMeasuredBasePosition()
	{
		return getCommandedPosition();
	}

	KDL::Twist RobotBaseDriverSimulation::getMeasuredBaseVelocity()
	{
		return getCommandedVelocity();
	}

	KDL::Twist RobotBaseDriverSimulation::getCommandedBaseVelocity()
	{
		if(CartesianPositionDevice::cmdTime < lastVel) {
			return vel;
		} else {
			return getCommandedVelocity();
		}
	}

	int RobotBaseDriverSimulation::checkPosition(KDL::Frame position)
	{
		if(fabs(position.p.z()) > 1e-3 ||
				fabs(position.M.UnitZ().x()) > 1e-3  ||
				fabs(position.M.UnitZ().y()) > 1e-3) {
			return 1;
		}
		return 0;
	}
	KDL::Frame RobotBaseDriverSimulation::getMeasuredPosition()
	{
		return getCommandedPosition();
	}
	KDL::Twist RobotBaseDriverSimulation::getMeasuredVelocity()
	{
		return getCommandedVelocity();
	}

	int RobotBaseDriverSimulation::getWheelCount() const
	{
		return 0;
	}
	double RobotBaseDriverSimulation::getWheelPosition(int wheel) const
	{
		return 0;
	}
	double RobotBaseDriverSimulation::getWheelVelocity(int wheel) const
	{
		return 0;
	}

} /* namespace robotbase */
