/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef SRC_LIBS_KUKA_KR_SIMULATION_KR_ARM_SIM_HPP_
#define SRC_LIBS_KUKA_KR_SIMULATION_KR_ARM_SIM_HPP_

#include <string>
#include <rcc/Device.hpp>
#include <rcc/DeviceInstanceT.hpp>
#include <libs/framework/io/simulation/IODeviceSimulation.hpp>
#include "../device/KR_Arm_Device.hpp"

#include "KR_Controller_Sim.hpp"

namespace kuka_kr
{

	class KR_Arm_Sim: virtual public KR_Arm_Device, virtual public IO::IODeviceSimulation
	{
	public:
		static const std::string kr_arm_sim_devicename;

		KR_Arm_Sim(std::string name, RPI::parameter_t parameters);
		virtual ~KR_Arm_Sim();

		static KR_Arm_Sim* createDevice(std::string name, RPI::parameter_t parameters);
	private:
		KR_Controller_Sim* sim;
	};

} /* namespace kuka_kr */

#endif /* SRC_LIBS_KUKA_KR_SIMULATION_KR_ARM_SIM_HPP_ */
