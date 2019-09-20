/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef KR_ARM_RSI_HPP_
#define KR_ARM_RSI_HPP_

#include <rcc/Device.hpp>
#include <rcc/DeviceInstanceT.hpp>

#include "../device/KR_Arm_Device.hpp"
#include "KR_Controller_RSI.hpp"
#include "../kinematics/KR_Kin.hpp"
#include "../device/KR_Controller.hpp"
#include <rtt/TaskContext.hpp>

namespace kuka_kr
{
	class KR_Arm_RSI: public KR_Arm_Device
	{
	public:
		const static std::string kr_devicename;

		KR_Arm_RSI(std::string name, RPI::parameter_t parameters);
		virtual ~KR_Arm_RSI();

		static KR_Arm_RSI* createDevice(std::string name, RPI::parameter_t parameters);

		virtual void setAnalogOut(int port, double value);
		virtual double getAnalogOut(int port);
		virtual double getAnalogIn(int port);

		virtual void setDigitalOut(int port, bool value);
		virtual bool getDigitalOut(int port);
		virtual bool getDigitalIn(int port);

		virtual unsigned int getNumDigitalIn() const;
		virtual unsigned int getNumDigitalOut() const;
		virtual unsigned int getNumAnalogIn() const;
		virtual unsigned int getNumAnalogOut() const;
	private:
		KR_Controller_RSI* rsi;

		static const KR_to_SI_factors getConvFactors();
	};

} /* namespace kuka_kr */

#endif /* KR_ARM_RSI_HPP_ */
