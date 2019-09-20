/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EL2008.hpp"

namespace beckhoffio
{
	using namespace RPI;
	using namespace std;

	EL2008::EL2008(std::string name, parameter_t parameters) :
			AbstractBeckhoffIODevice(name, parameters)
	{
		process_data = 0;
	}

	EL2008::~EL2008()
	{
	}

	EL2008* EL2008::createDevice(string name, parameter_t parameters)
	{
		EL2008* ret = new EL2008(name, parameters);
		return ret;
	}

	bool EL2008::getDigitalOut(int port)
	{
		if (port < 0 || port > 7)
			return false;

		return (process_data >> port) & 0x1;
	}

	void EL2008::setDigitalOut(int port, bool value)
	{
		if (port < 0 || port > 7)
			return;

		if (value)
			process_data = process_data | (1 << port);
		else
			process_data = process_data & ~(1 << port);
	}

	void EL2008::readProcessData(uint8* data, uint32 size)
	{

	}

	void EL2008::writeProcessData(uint8* data, uint32 size)
	{
		if (size != 1)
		{

			return;
		}

		*data = process_data;

	}

	unsigned int EL2008::getNumDigitalOut() const
	{
		return 8;
	}
}
