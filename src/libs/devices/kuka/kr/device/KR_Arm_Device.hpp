/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef SRC_LIBS_KUKA_KR_DEVICE_KR_ARM_DEVICE_HPP_
#define SRC_LIBS_KUKA_KR_DEVICE_KR_ARM_DEVICE_HPP_

#include "KR_Robot_Device.hpp"

#include <libs/framework/io/interface/IOInterface.hpp>

namespace kuka_kr
{

	class RTT_EXPORT KR_Arm_Device: public KR_Robot_Device, virtual public IO::IOInterface
	{
	public:
		KR_Arm_Device(std::string name, RPI::parameter_t parameter, KR_Controller* controller);
		virtual ~KR_Arm_Device();

	protected:
		dh_parameters dh_param;
	};

} /* namespace kuka_kr */

#endif /* SRC_LIBS_KUKA_KR_DEVICE_KR_ARM_DEVICE_HPP_ */
