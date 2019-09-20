/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "IODeviceSimulation.hpp"
#include "string.h"

namespace IO
{

	IODeviceSimulation::IODeviceSimulation(unsigned int ndigout, unsigned int nanaout)
	{
		this->numdigout = ndigout;
		this->numanaout = nanaout;

		this->digout = new bool[numdigout];
		this->anaout = new double[numanaout];

		for (int i = 0; i < numdigout; ++i)
			digout[i] = false;
		for (int i = 0; i < numanaout; ++i)
			anaout[i] = 0;
	}

	IODeviceSimulation::~IODeviceSimulation()
	{
		delete[] digout;
		delete[] anaout;
	}

	void IODeviceSimulation::setDigitalOut(int port, bool value)
	{
		if (port < 0 || port >= numdigout)
			return;
		digout[port] = value;
	}
	bool IODeviceSimulation::getDigitalOut(int port)
	{
		if (port < 0 || port >= numdigout)
			return false;
		return digout[port];
	}
	bool IODeviceSimulation::getDigitalIn(int port)
	{
		return false;
	}
	void IODeviceSimulation::setAnalogOut(int port, double value)
	{
		if (port < 0 || port >= numanaout)
			return;
		anaout[port] = value;
	}

	double IODeviceSimulation::getAnalogOut(int port)
	{
		if (port < 0 || port >= numanaout)
			return 0;
		return anaout[port];
	}
	double IODeviceSimulation::getAnalogIn(int port)
	{
		return 0;
	}

	unsigned int IODeviceSimulation::getNumDigitalIn() const
	{
		return 0;
	}
	unsigned int IODeviceSimulation::getNumDigitalOut() const
	{
		return numdigout;
	}
	unsigned int IODeviceSimulation::getNumAnalogIn() const
	{
		return 0;
	}
	unsigned int IODeviceSimulation::getNumAnalogOut() const
	{
		return numanaout;
	}


} /* namespace io */
