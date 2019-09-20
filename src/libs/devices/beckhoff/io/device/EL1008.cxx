/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EL1008.hpp"

#include <rtt/Logger.hpp>

namespace beckhoffio
{
	using namespace RPI;
	using namespace std;

	EL1008::EL1008(std::string name, parameter_t parameters) :
			AbstractBeckhoffIODevice(name, parameters)
	{
		process_data = 0;
	}

	EL1008::~EL1008()
	{
	}

	EL1008* EL1008::createDevice(string name, parameter_t parameters)
	{
		EL1008* ret = new EL1008(name, parameters);
		return ret;
	}

	bool EL1008::getDigitalIn(int port)
	{
		if(port < 0 || port > 7)
			return false;

		return (process_data >> port) & 0x1;
	}

	void EL1008::readProcessData(uint8* data, uint32 size)
	{
		if(size != 1)
			return;

		process_data = *data;
	}

	void EL1008::writeProcessData(uint8* data, uint32 size)
	{

	}

	unsigned int EL1008::getNumDigitalIn() const {
		return 8;
	}

}
