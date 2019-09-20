/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef EL4004_HPP_
#define EL4004_HPP_

#include "AbstractBeckhoffIODevice.hpp"

#include <libs/framework/io/interface/IOInterface.hpp>
#include <rcc/Device.hpp>

namespace beckhoffio
{

	const std::string dev_el4004 = "el4004";

#ifdef WIN32
#define PACKED
#pragma pack(push,1)
#else
#define PACKED __attribute__((packed))
#endif

	struct el4004_pd
	{
		int16 output[4];
	}PACKED;

#ifdef WIN32
#pragma pack(pop)
#endif
	/**
	 * device for EL4004, 4 channel analog output
	 */
	class EL4004: public AbstractBeckhoffIODevice
	{
	public:
		EL4004(std::string name, RPI::parameter_t parameters);
		virtual ~EL4004();

		static EL4004* createDevice(std::string name, RPI::parameter_t parameters);

		virtual void writeProcessData(uint8*, uint32);
		virtual void readProcessData(uint8*, uint32);

		virtual double getAnalogOut(int port);
		virtual void setAnalogOut(int port, double value);

		virtual unsigned int getNumAnalogOut() const;
	private:
		static const int channels = 4;

		double output[channels];
	};

} /* namespace beckhoffio */
#endif /* EL4004_HPP_ */
