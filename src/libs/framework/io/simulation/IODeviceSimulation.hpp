/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef IODEVICESIMULATION_HPP_
#define IODEVICESIMULATION_HPP_

#include "../interface/IOInterface.hpp"
#include <rtt/rtt-config.h>

namespace IO
{

	class RTT_EXPORT IODeviceSimulation: virtual public IO::IOInterface
	{
	public:
		IODeviceSimulation(unsigned int digout, unsigned int anaout);
		virtual ~IODeviceSimulation();

		virtual void setDigitalOut(int port, bool value);
		virtual bool getDigitalOut(int port);
		virtual bool getDigitalIn(int port);
		virtual void setAnalogOut(int port, double value);
		virtual double getAnalogOut(int port);
		virtual double getAnalogIn(int port);

		virtual unsigned int getNumDigitalIn() const;
		virtual unsigned int getNumDigitalOut() const;
		virtual unsigned int getNumAnalogIn() const;
		virtual unsigned int getNumAnalogOut() const;

	private:
		unsigned int numdigout, numanaout;
		bool *digout;
		double *anaout;
	};

} /* namespace io */
#endif /* IODEVICESIMULATION_HPP_ */
