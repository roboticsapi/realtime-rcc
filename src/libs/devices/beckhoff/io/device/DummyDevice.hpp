/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DUMMY_HPP_
#define DUMMY_HPP_

#include "AbstractBeckhoffIODevice.hpp"

#include <libs/framework/io/interface/IOInterface.hpp>
#include <rcc/Device.hpp>

namespace beckhoffio
{
	const std::string dev_dummy = "ec_dummy";

	/**
	 * The Dummy Device simply absorbs an unused EtherCAT device. The EK1100 is an
	 * example for an EtherCAT slave which does not need any configuration, does not
	 * have any process data available, but should still be claimed in order for
	 * the bus to go to operational mode.
	 */
	class DummyDevice: public AbstractBeckhoffIODevice
	{
	public:
		DummyDevice(std::string name, RPI::parameter_t parameters);
		virtual ~DummyDevice();

		static DummyDevice* createDevice(std::string name, RPI::parameter_t parameters);

		void writeProcessData(uint8*, uint32);
		void readProcessData(uint8*, uint32);
	};
}
#endif /* DUMMY_HPP_ */
