/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef EL2008_HPP_
#define EL2008_HPP_

#include "AbstractBeckhoffIODevice.hpp"

#include <libs/framework/io/interface/IOInterface.hpp>
#include <rcc/Device.hpp>


namespace beckhoffio
{
	const std::string dev_el2008 = "el2008";

	/**
	 * device for EL2008, 8 channel digital output
	 */
	class EL2008: public AbstractBeckhoffIODevice
	{
	public:
		EL2008(std::string name, RPI::parameter_t parameters);
		virtual ~EL2008();

		static EL2008* createDevice(std::string name, RPI::parameter_t parameters);

		virtual void writeProcessData(uint8*, uint32);
		virtual void readProcessData(uint8*, uint32);

		virtual bool getDigitalOut(int port);
		virtual void setDigitalOut(int port, bool state);

		virtual unsigned int getNumDigitalOut() const;

	private:
		uint8 process_data;

	};

} /* namespace beckhoffio */
#endif /* EL2008_HPP_ */
