/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "DummyDevice.hpp"

namespace beckhoffio
{
	using namespace RPI;
	using namespace std;

	DummyDevice::DummyDevice(std::string name, parameter_t parameters) :
			AbstractBeckhoffIODevice(name, parameters)
	{
		// TODO Auto-generated constructor stub

	}

	DummyDevice::~DummyDevice()
	{
		// TODO Auto-generated destructor stub
	}

	DummyDevice* DummyDevice::createDevice(string name, parameter_t parameters)
	{
		DummyDevice* ret = new DummyDevice(name, parameters);
		return ret;
	}

	void DummyDevice::readProcessData(uint8* data, uint32 size)
	{

	}

	void DummyDevice::writeProcessData(uint8* data, uint32 size)
	{

	}

}
