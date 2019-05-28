/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EL3064.hpp"

namespace beckhoffio
{

	using namespace RPI;
	using namespace std;

	EL3064::EL3064(std::string name, parameter_t parameters) :
			AbstractBeckhoffIODevice(name, parameters)
	{
		for (int i = 0; i < channels; i++)
		{
			input[i] = 0;
		}
	}

	EL3064::~EL3064()
	{
	}

	EL3064* EL3064::createDevice(string name, parameter_t parameters)
	{
		EL3064* ret = new EL3064(name, parameters);
		return ret;
	}

	double EL3064::getAnalogIn(int port)
	{
		if (port < 0 || port >= channels)
			return 0;

		return input[port];
	}

	void EL3064::readProcessData(uint8* data, uint32 size)
	{
		el3064_pd process_data;

		if (size != sizeof(process_data))
			return;

		process_data = *((el3064_pd*) data);

		for (int i = 0; i < channels; ++i)
		{
			input[i] = (process_data.input[i].value / (double) 0x7fff) * 10.0;
		}

	}

	void EL3064::writeProcessData(uint8* data, uint32 size)
	{

	}

	unsigned int EL3064::getNumAnalogIn() const
	{
		return 4;
	}

} /* namespace beckhoffio */
