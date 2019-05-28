/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EL2024.hpp"

namespace beckhoffio
{
	using namespace RPI;
	using namespace std;

	EL2024::EL2024(std::string name, parameter_t parameters) :
			AbstractBeckhoffIODevice(name, parameters)
	{
		process_data = 0;
	}

	EL2024::~EL2024()
	{
	}

	EL2024* EL2024::createDevice(string name, parameter_t parameters)
	{
		EL2024* ret = new EL2024(name, parameters);
		return ret;
	}

	bool EL2024::getDigitalOut(int port)
	{
		if (port < 0 || port > 3)
			return false;

		return (process_data >> port) & 0x1;
	}

	void EL2024::setDigitalOut(int port, bool value)
	{
		if (port < 0 || port > 3)
			return;

		if (value)
			process_data = process_data | (1 << port);
		else
			process_data = process_data & ~(1 << port);
	}

	void EL2024::readProcessData(uint8* data, uint32 size)
	{

	}

	void EL2024::writeProcessData(uint8* data, uint32 size)
	{
		/*std::cout << size << std::endl;
		if (size != 1)
		{

			return;
		}

		*data = process_data;*/
		//*data = *data | 3;

	}
	void EL2024::writeProcessDataBit(uint8* data, uint16 size, uint8 startbit)
	{
		//std::cout << "Data: " << (uint16) data << " size: " << size << " start: " << (int) startbit << std::endl;

		uint8 mask = ~(15 << startbit);
		uint8 copy = *data & mask;
		copy = copy | (process_data << startbit);
		*data = copy;


	}


	unsigned int EL2024::getNumDigitalOut() const
	{
		return 4;
	}
}
