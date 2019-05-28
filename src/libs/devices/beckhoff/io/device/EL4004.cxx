/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EL4004.hpp"

namespace beckhoffio
{

	using namespace RPI;
	using namespace std;

	EL4004::EL4004(std::string name, parameter_t parameters) :
			AbstractBeckhoffIODevice(name, parameters)
	{
		for (int i = 0; i < channels; i++)
		{
			output[i] = 0;
		}
	}

	EL4004::~EL4004()
	{
	}

	EL4004* EL4004::createDevice(string name, parameter_t parameters)
	{
		EL4004* ret = new EL4004(name, parameters);
		return ret;
	}

	void EL4004::setAnalogOut(int port, double value)
	{
		if (port < 0 || port >= channels)
			return;

		if (value > 10.0 || value < 0.0)
			return;

		output[port] = value;
	}

	double EL4004::getAnalogOut(int port)
	{
		if (port < 0 || port >= channels)
			return 0;

		return output[port];
	}

	void EL4004::readProcessData(uint8* data, uint32 size)
	{

	}

	void EL4004::writeProcessData(uint8* data, uint32 size)
	{
		el4004_pd process_data;

		if (size != sizeof(process_data))
			return;

		for (int i = 0; i < channels; ++i)
		{
			double fact = output[i] / 10.0;
			if (fact > 1.0)
				fact = 1.0;
			process_data.output[i] = fact * 0x7fff;
		}

		*((el4004_pd*) data) = process_data;
	}

	unsigned int EL4004::getNumAnalogOut() const
	{
		return 4;
	}

} /* namespace beckhoffio */
