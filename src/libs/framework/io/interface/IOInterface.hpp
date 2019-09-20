/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef IODriver_HPP_
#define IODriver_HPP_

#include <rtt/rtt-config.h>

/**
 * \addtogroup IODriver
 * @{
 */

/**
 * \brief Driver supporting digital and analog I/O ports
 */
namespace IO
{
	/**
	 * abstract driver interface
	 * \author Michael Vistein
	 */
	class RTT_EXPORT IOInterface
	{
	public:
		virtual ~IOInterface() { }

		/**
		 * sets a digital output
		 * \param port Number of port, 0 based
		 * \param value Value to set port to
		 */
		virtual void setDigitalOut(int port, bool value) = 0;

		/**
		 * reads current value of digital output port
		 * \param port Number of port, 0 based
		 * \return Current value of digital output port
		 */
		virtual bool getDigitalOut(int port) = 0;

		/**
		 * reads current value of digital input port
		 * \param port Number of port, 0 based
		 * \return Current value of digital input port
		 */
		virtual bool getDigitalIn(int port) = 0;

		/**
		 * sets an aanlog output
		 * \param port Number of port, 0 based
		 * \param value Value to set port to
		 */
		virtual void setAnalogOut(int port, double value) = 0;

		/**
		 * reads current value of analog output port
		 * \param port Number of port, 0 based
		 * \return Current value of analog output port
		 */
		virtual double getAnalogOut(int port) = 0;

		/**
		 * reads current value of analog input port
		 * \param port Number of port, 0 based
		 * \return Current value of analog input port
		 */
		virtual double getAnalogIn(int port) = 0;

		virtual unsigned int getNumDigitalIn() const = 0;
		virtual unsigned int getNumDigitalOut() const = 0;
		virtual unsigned int getNumAnalogIn() const = 0;
		virtual unsigned int getNumAnalogOut() const = 0;
	};
}

/**
 * }@
 */

#endif /* IODriver_HPP_ */
