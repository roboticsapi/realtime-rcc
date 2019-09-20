/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ABSTRACTBECKHOFFIODEVICE_HPP_
#define ABSTRACTBECKHOFFIODEVICE_HPP_

#include <libs/framework/io/interface/IOInterface.hpp>
#include <libs/framework/ethercat/interface/CoEEthercatSlave.hpp>
#include <libs/framework/ethercat/interface/EthercatMaster.hpp>
#include <rcc/Device.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/DeviceInstanceT.hpp>

namespace beckhoffio
{

	class AbstractBeckhoffIODevice: public RPI::Device,
			public IO::IOInterface,
			public ethercat::CoEEthercatSlave,
			public ethercat::EthercatDevice
	{
	public:
		AbstractBeckhoffIODevice(std::string name, RPI::parameter_t parameters);
		virtual ~AbstractBeckhoffIODevice();

		virtual void setEStop(bool estop);
		virtual std::set<std::string> getMutableParameters() const;
		virtual void updateParameters();
		virtual RPI::DeviceState getDeviceState() const;

		virtual void inSafeOp();
		virtual void inPreOp();

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


		virtual void startupDevice();
		virtual void shutdownDevice();
		virtual void updateDevice();
	private:
		bool isEStop;
		bool inOp;

		RPI::DeviceInstanceT<ethercat::EthercatMaster> ecmaster;
	};

} /* namespace beckhoffio */
#endif /* ABSTRACTBECKHOFFIODEVICE_HPP_ */
