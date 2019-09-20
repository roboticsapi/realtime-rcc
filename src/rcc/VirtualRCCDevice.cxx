/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "VirtualRCCDevice.hpp"
#include <boost/lexical_cast.hpp>
#include "TypeKitT.hpp"
#include "Registry.hpp"

namespace RPI
{

	VirtualRCCDevice::VirtualRCCDevice(std::string name, RPI::parameter_t parameter) :
			Device(name, parameter),
			RTT::os::Thread(ORO_SCHED_RT, RTT::os::HighestPriority, VRCC_CYCLE_TIME, 0, "vrcc"),
			lastEStopEvent(0), lastEStopTriggered(0), isEStop(false), needEStop(false)
	{
		// Check and set default parameters
		parameter_t defaults;
		defaults["netunloading"] = "true";
		defaults["debugginglevel"] = "1";
		defaults["debuggingtime"] = "2.0";
		defaults["needestop"] = "false";
		defaults["webport"] = "8080";

		merge_default_parameters(this->parameters, defaults);

		updateParameters();

		webport = this->parameters["webport"];

		this->start();
	}

	VirtualRCCDevice::~VirtualRCCDevice()
	{
	}

	std::set<std::string> VirtualRCCDevice::getMutableParameters() const
	{
		std::set<std::string> ret;
		ret.insert("netunloading");
		ret.insert("debugginglevel");
		ret.insert("debuggingtime");
		ret.insert("needestop");

		return ret;
	}

	void VirtualRCCDevice::updateParameters()
	{
		netUnloading = (parameters["netunloading"] != "false");
		try
		{
			debuggingLevel = boost::lexical_cast<int>(parameters["debugginglevel"]);
		} catch (boost::bad_lexical_cast&)
		{
		}

		try
		{
			debuggingTime = boost::lexical_cast<double>(parameters["debuggingtime"]);
		} catch (boost::bad_lexical_cast&)
		{

		}

		TypeKits::getInstance()->getTypeKit<bool>()->fromString(&needEStop, parameters["needestop"]);

	}

	bool VirtualRCCDevice::getNetUnloading() const
	{
		return netUnloading;
	}

	int VirtualRCCDevice::getDebuggingLevel() const
	{
		return debuggingLevel;
	}

	double VirtualRCCDevice::getDebuggingTime() const
	{
		return debuggingTime;
	}

	bool VirtualRCCDevice::getEStop() const
	{
		return isEStop;
	}

	void VirtualRCCDevice::setEStop(bool value)
	{
		if (value)
		{
			if (!isEStop)
			{
				isEStop = true;
				notifyDevices();
			}
			lastEStopTriggered = RTT::os::TimeService::Instance()->getNSecs();
		}
		lastEStopEvent = RTT::os::TimeService::Instance()->getNSecs();
	}

	std::string VirtualRCCDevice::getWebPort() const
	{
		return webport;
	}

	bool VirtualRCCDevice::initialize()
	{
		return true;
	}

	void VirtualRCCDevice::step()
	{
		RTT::os::TimeService::nsecs lasteventdiff = RTT::os::TimeService::Instance()->getNSecs(lastEStopEvent);
		RTT::os::TimeService::nsecs lastestopdiff = RTT::os::TimeService::Instance()->getNSecs(lastEStopTriggered);

		// running EStop module is required, but does not seem to exist -> EStop!
		if (needEStop && (lasteventdiff) > 2 * VRCC_CYCLE_TIME * 1e9)
		{
			lastEStopTriggered = RTT::os::TimeService::Instance()->getNSecs();
			if (!isEStop)
			{
				isEStop = true;
				notifyDevices();
			}
		} else if (isEStop && (lastestopdiff) > 2 * VRCC_CYCLE_TIME * 1e9)
		{
			isEStop = false;
			notifyDevices();
		}
	}

	void VirtualRCCDevice::finalize()
	{

	}

	void VirtualRCCDevice::notifyDevices()
	{
		devicelist_t devices = Registry::getRegistry()->getDevices();
		for (devicelist_t::const_iterator it = devices.begin(); it != devices.end(); ++it)
		{
			Device* dev = Registry::getRegistry()->getDevice(it->name);
			dev->setEStop(isEStop);
		}
	}

	DeviceState VirtualRCCDevice::getDeviceState() const
	{
		return getEStop() ? DeviceState::SAFE_OPERATIONAL : DeviceState::OPERATIONAL;
	}

} /* namespace RPI */
