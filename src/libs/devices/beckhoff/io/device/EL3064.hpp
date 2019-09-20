/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef EL3064_HPP_
#define EL3064_HPP_

#include "AbstractBeckhoffIODevice.hpp"

#include <libs/framework/io/interface/IOInterface.hpp>
#include <rcc/Device.hpp>

namespace beckhoffio
{

	const std::string dev_el3064 = "el3064";

#ifdef WIN32
#define PACKED
#pragma pack(push,1)
#else
#define PACKED __attribute__((packed))
#endif

	struct el3064_inp_pd
	{
		bool underrange :1;
		bool overrange :1;
		uint8 limit1 :2;
		uint8 limit2 :2;
		bool error :1;
		bool fill1 :1;
		uint8 fill2 :6;
		bool txpdo_state :1;
		bool txpdo_toggle :1;
		int16 value;
	}PACKED;

	struct el3064_pd
	{
		el3064_inp_pd input[4];
	}PACKED;

#ifdef WIN32
#pragma pack(pop)
#endif
	/**
	 * device for EL3064, 4 channel analog input
	 */
	class EL3064: public AbstractBeckhoffIODevice
	{
	public:
		EL3064(std::string name, RPI::parameter_t parameters);
		virtual ~EL3064();

		static EL3064* createDevice(std::string name, RPI::parameter_t parameters);

		virtual void writeProcessData(uint8*, uint32);
		virtual void readProcessData(uint8*, uint32);

		virtual double getAnalogIn(int port);

		virtual unsigned int getNumAnalogIn() const;

	private:
		static const int channels = 4;

		double input[channels];
	};

} /* namespace beckhoffio */
#endif /* EL3064_HPP_ */
