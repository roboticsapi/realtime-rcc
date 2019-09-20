/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef SRC_LIBS_KUKA_KR_DEVICE_KR_CONTROLLER_HPP_
#define SRC_LIBS_KUKA_KR_DEVICE_KR_CONTROLLER_HPP_

#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <rcc/Device.hpp>

namespace kuka_kr
{

	class RTT_EXPORT KR_Controller: public virtual robotarm::RobotArmInterface, public virtual RPI::Device
	{

	public:
		KR_Controller();
		virtual ~KR_Controller();

		// Configuration of maxvel and maxacc
		virtual void setJointConfig(int axis, double max_vel, double max_acc, double minj, double maxj) = 0;
		virtual void setInitialToolConfig(double mass, KDL::Vector com) = 0;
	};

}
/* namespace kuka_kr */

#endif /* SRC_LIBS_KUKA_KR_DEVICE_KR_CONTROLLER_HPP_ */
