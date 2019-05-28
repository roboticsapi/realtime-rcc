/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "KR_Controller_RSI.hpp"
#include "KR_Arm_RSI.hpp"
#include <rtt/Activity.hpp>
#include <rcc/rapidxml/rapidxml.hpp>
#include <cstdlib>
#include <iomanip>

namespace kuka_kr
{
	const std::string KR_Arm_RSI::kr_devicename = "kuka_kr_arm_rsi";

	KR_Arm_RSI::KR_Arm_RSI(std::string name, RPI::parameter_t parameters) :
			KR_Arm_Device(name, parameters,
					rsi = new KR_Controller_RSI(name, getParameter("ethernet"), getConvFactors())), Device(name,
					parameters)
	{
		rsi->configure();
		rsi->start();

		for (const auto& dumper : rsi->getCrashDumpers())
			addCrashDumper(dumper);


	}

	KR_Arm_RSI::~KR_Arm_RSI()
	{
		rsi->stop();
		rsi->cleanup();

		delete rsi;
	}

	KR_Arm_RSI* KR_Arm_RSI::createDevice(std::string name, RPI::parameter_t parameters)
	{
		return new KR_Arm_RSI(name, parameters);
	}

	const KR_to_SI_factors KR_Arm_RSI::getConvFactors()
	{
		KR_to_SI_factors factors;
		for(int i = 0; i < 6; i++)
			factors.conv_factor[i] = KDL::deg2rad;

		for(int i = 6; i < 12; i++)
			factors.conv_factor[i] = 0.001;

		return factors;
	}

	// No analog I/O for the moment
	void KR_Arm_RSI::setAnalogOut(int port, double value)
	{

	}

	double KR_Arm_RSI::getAnalogOut(int port)
	{
		return 0;
	}

	double KR_Arm_RSI::getAnalogIn(int port)
	{
		return 0;
	}


	unsigned int KR_Arm_RSI::getNumDigitalIn() const
	{
		return 4;
	}
	unsigned int KR_Arm_RSI::getNumDigitalOut() const
	{
		return 4;
	}
	unsigned int KR_Arm_RSI::getNumAnalogIn() const
	{
		return 0;
	}
	unsigned int KR_Arm_RSI::getNumAnalogOut() const
	{
		return 0;
	}

	void KR_Arm_RSI::setDigitalOut(int port, bool value)
	{
		if(port >= 0 && port < kr_rsi_digout)
			rsi->getRecvData()->digout[port] = value;
	}
	bool KR_Arm_RSI::getDigitalOut(int port)
	{
		if(port >= 0 && port < kr_rsi_digout)
			return rsi->getRecvData()->digout[port];
		return false;

	}
	bool KR_Arm_RSI::getDigitalIn(int port)
	{
		if(port >= 0 && port < kr_rsi_digin)
			return rsi->getRecvData()->digin[port];
		return false;

	}

}
/* namespace kuka_kr */
