/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotArmEthercat.hpp"
#include "youBotGripperEthercat.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/units/cmath.hpp>

#include <rtt/Activity.hpp>
#include <kdl/utilities/utility.h>

#include <rcc/Device.hpp>
#include <rcc/DeviceFactory.hpp>
#include "EthercatProtocolDefinitions.hpp"


#define YB_CYCLE_TIME 0.002

namespace kuka_youbot
{
	using namespace std;
	using namespace RPI;
	using namespace boost::units;

	youBotGripperEthercat::youBotGripperEthercat(string name, parameter_t parameters) :
			TaskContext("youbot_gripper_" + name, PreOperational), Device(name, parameters),
			arm(parameters["arm"]),
			gripperState(Gripper_Initialize),
			maxGripperDistance(0.023), gripperOffset(0), maxEncoderValueGripper(67000), gripperGoal(0)
	{
		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0,  name));
	}


	youBotGripperEthercat::~youBotGripperEthercat()
	{
	}

	bool youBotGripperEthercat::configureHook()
	{
		return true;
	}

	bool youBotGripperEthercat::startHook()
	{
		trigger();
		return true;
	}

	void youBotGripperEthercat::stopHook()
	{
		return;
	}

	void youBotGripperEthercat::cleanupHook()
	{
		return;
	}

	void youBotGripperEthercat::updateHook()
	{
		if(arm.getDevice() == 0) return;
		switch(gripperState) {
		case Gripper_Initialize:
			if(arm->getDeviceState() != DeviceState::OFFLINE) {
				RTT::log(RTT::Info) << "Initializing gripper." << RTT::endlog();
				arm->calibrateGripperBlocking();
				RTT::log(RTT::Info) << "Gripper initialized." << RTT::endlog();
				gripperState = Gripper_Wait;
			} else {
				usleep(500);
				trigger();
			}
			break;
		case Gripper_Move:
			arm->setGripperPositionBlocking(gripperGoal);
			gripperState = Gripper_Wait;
			break;
		case Gripper_Wait:
			break;
		}

	}

	//value is in meters
	bool youBotGripperEthercat::setGripperPosition(float value)
	{
		if(gripperState != Gripper_Wait)
				return false;

		if (!checkGripperPositionInRange(value))
				return false;

		gripperGoal = value;
		gripperState = Gripper_Move;
		trigger();
		return true;
	}

	//value is in meters
	float youBotGripperEthercat::getGripperPosition()
	{
		return gripperGoal;
	}


	bool youBotGripperEthercat::gripperBusy()
	{
		return gripperState != Gripper_Wait;
	}

	void youBotGripperEthercat::updateParameters()
	{
		return;
	}

	set<string> youBotGripperEthercat::getMutableParameters() const
	{
		return set<string>();
	}

	youBotGripperEthercat* youBotGripperEthercat::createDevice(string name, parameter_t parameters)
	{
		youBotGripperEthercat* ret = new youBotGripperEthercat(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	void youBotGripperEthercat::setEStop(bool estop)
	{
	}


	bool youBotGripperEthercat::checkGripperPositionInRange(const float newPosition) const
	{
		const float newPos_roundedUp = (ceilf(newPosition * 1000) / 1000);
		const float upperRange = (ceilf((maxGripperDistance + gripperOffset) * 1000) / 1000);

		if (newPosition < 0 || newPos_roundedUp > upperRange)
		{
			return false;
		}
		return true;
	}

	RPI::DeviceState youBotGripperEthercat::getDeviceState() const
	{
		if(arm.getDevice() == 0) return RPI::DeviceState::OFFLINE;
		return arm->getDeviceState();
	}

} //End of namespace yb
