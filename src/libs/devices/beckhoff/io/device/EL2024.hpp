/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef EL2024_HPP_
#define EL2024_HPP_

#include "AbstractBeckhoffIODevice.hpp"

#include <libs/framework/io/interface/IOInterface.hpp>
#include <rcc/Device.hpp>


namespace beckhoffio
{
	const std::string dev_el2024 = "el2024";

	/**
	 * device for EL2024, 4 channel digital output
	 */
	class EL2024: public AbstractBeckhoffIODevice
	{
	public:
		EL2024(std::string name, RPI::parameter_t parameters);
		virtual ~EL2024();

		static EL2024* createDevice(std::string name, RPI::parameter_t parameters);

		virtual void writeProcessData(uint8*, uint32);
		virtual void writeProcessDataBit(uint8*, uint16, uint8);
		virtual void readProcessData(uint8*, uint32);

		virtual bool getDigitalOut(int port);
		virtual void setDigitalOut(int port, bool state);

		virtual unsigned int getNumDigitalOut() const;

	private:
		uint8 process_data;

	};

} /* namespace beckhoffio */
#endif /* EL2024_HPP_ */
