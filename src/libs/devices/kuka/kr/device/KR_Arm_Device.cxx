/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "KR_Arm_Device.hpp"

namespace kuka_kr
{

	KR_Arm_Device::KR_Arm_Device(std::string name, RPI::parameter_t parameter, KR_Controller* controller) :
			KR_Robot_Device(name, parameter, controller), RPI::Device(name, parameter)
	{
		// Kinematics
		dh_param.dh_d = RPI::Module::parseString<double>(getParameter("dh_d"));
		dh_param.dh_t = RPI::Module::parseString<double>(getParameter("dh_t"));
		dh_param.dh_a = RPI::Module::parseString<double>(getParameter("dh_a"));
		dh_param.dh_al = RPI::Module::parseString<double>(getParameter("dh_al"));

		KDL::JntArray q_min(minj.size()), q_max(maxj.size());

		for (int i = 0; i < minj.size(); ++i)
			q_min(i) = minj[i];
		for (int i = 0; i < maxj.size(); ++i)
			q_max(i) = maxj[i];

		kincalc = new KR_Kin_Calculation(dh_param, q_min, q_max);
	}

	KR_Arm_Device::~KR_Arm_Device()
	{
		delete kincalc;
		kincalc = 0;
	}


} /* namespace kuka_kr */
