/*
 * KR_Arm_Sim.cxx
 *
 *  Created on: 28.11.2014
 *      Author: visteimi
 */

#include "KR_Arm_Sim.hpp"

namespace kuka_kr
{
	const std::string KR_Arm_Sim::kr_arm_sim_devicename = "kuka_kr_arm_sim";

	KR_Arm_Sim::KR_Arm_Sim(std::string name, RPI::parameter_t parameters) :
			KR_Arm_Device(name, parameters, sim = new KR_Controller_Sim(name)), Device(name, parameters), IODeviceSimulation(4,0)

	{
		// TODO Auto-generated constructor stub

	}

	KR_Arm_Sim::~KR_Arm_Sim()
	{
		delete sim;
	}

	KR_Arm_Sim* KR_Arm_Sim::createDevice(std::string name, RPI::parameter_t parameters)
	{
		return new KR_Arm_Sim(name, parameters);
	}


} /* namespace kuka_kr */
