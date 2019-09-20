/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef EL1008_HPP_
#define EL1008_HPP_

#include "AbstractBeckhoffIODevice.hpp"

#include <libs/framework/io/interface/IOInterface.hpp>
#include <rcc/Device.hpp>

namespace beckhoffio
{
	const std::string dev_el1008 = "el1008";

	/**
	 * device for EL1008, 8 channel digital input
	 */
	class EL1008: public AbstractBeckhoffIODevice
	{
	public:
		EL1008(std::string name, RPI::parameter_t parameters);
		virtual ~EL1008();

		static EL1008* createDevice(std::string name, RPI::parameter_t parameters);

		virtual void writeProcessData(uint8*, uint32);
		virtual void readProcessData(uint8*, uint32);

		virtual bool getDigitalIn(int port);

		virtual unsigned int getNumDigitalIn() const;

	private:
		uint8 process_data;
	};
}
#endif /* EL1008_HPP_ */
