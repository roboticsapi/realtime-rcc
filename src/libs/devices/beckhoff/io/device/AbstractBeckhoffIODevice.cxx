/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "AbstractBeckhoffIODevice.hpp"

#include <rtt/Logger.hpp>

namespace beckhoffio
{
	using namespace RPI;
	using namespace std;

	AbstractBeckhoffIODevice::AbstractBeckhoffIODevice(std::string name, RPI::parameter_t parameters) :
			Device(name, parameters)
	{
		isEStop = false;
		inOp = false;

		if (!ecmaster.fetchInstance(getParameter("ethercatdevice")))
		{
			RTT::log(RTT::Error) << "Failed to create Beckhoff IO Device, no EtherCAT device found" << RTT::endlog();
		} else
		{
			ecmaster->addDevice(this);
		}
	}

	void AbstractBeckhoffIODevice::startupDevice()
	{
		std::vector<ethercat::EthercatSlaveInfo> sln = ecmaster->getSlaveNames();

		int destslno = getParameterT<int>("slave", 0);

		if (destslno > 0 && destslno <= sln.size())
		{
			ecmaster->addSlave(destslno, this);
		} else
		{
			RTT::log(RTT::Error) << "Failed to create Beckhoff IO Device, no slave with given ID found"
					<< RTT::endlog();
		}

	}

	void AbstractBeckhoffIODevice::updateDevice()
	{

	}

	void AbstractBeckhoffIODevice::shutdownDevice()
	{
		ecmaster->removeSlave(this);
	}

	AbstractBeckhoffIODevice::~AbstractBeckhoffIODevice()
	{
		// TODO Auto-generated destructor stub
	}

	void AbstractBeckhoffIODevice::setEStop(bool estop)
	{
		isEStop = estop;
	}

	void AbstractBeckhoffIODevice::updateParameters()
	{

	}

	set<string> AbstractBeckhoffIODevice::getMutableParameters() const
	{
		return set<string>();
	}

	DeviceState AbstractBeckhoffIODevice::getDeviceState() const
	{
		return (!isEStop && inOp) ? DeviceState::OPERATIONAL : DeviceState::SAFE_OPERATIONAL;
	}

	void AbstractBeckhoffIODevice::inSafeOp()
	{
		ecmaster->doSlaveOp(slaveno);
		inOp = true;
	}

	void AbstractBeckhoffIODevice::inPreOp()
	{
	}

	void AbstractBeckhoffIODevice::setDigitalOut(int port, bool value)
	{

	}
	bool AbstractBeckhoffIODevice::getDigitalOut(int port)
	{
		return false;
	}
	bool AbstractBeckhoffIODevice::getDigitalIn(int port)
	{
		return false;
	}
	void AbstractBeckhoffIODevice::setAnalogOut(int port, double value)
	{

	}
	double AbstractBeckhoffIODevice::getAnalogOut(int port)
	{
		return 0;
	}
	double AbstractBeckhoffIODevice::getAnalogIn(int port)
	{
		return 0;
	}

	unsigned int AbstractBeckhoffIODevice::getNumDigitalIn() const
	{
		return 0;
	}
	unsigned int AbstractBeckhoffIODevice::getNumDigitalOut() const
	{
		return 0;
	}
	unsigned int AbstractBeckhoffIODevice::getNumAnalogIn() const
	{
		return 0;
	}
	unsigned int AbstractBeckhoffIODevice::getNumAnalogOut() const
	{
		return 0;
	}

} /* namespace beckhoffio */
