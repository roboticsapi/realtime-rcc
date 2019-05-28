/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ROBOTBASESIMULATION_HPP_
#define ROBOTBASESIMULATION_HPP_

#include "../simulation/RobotBaseDriverSimulation.hpp"
#include <rcc/Device.hpp>

namespace robotbase
{
	const std::string dev_robotbase_sim = "robotbase_sim";

	class RobotBaseSim: public robotbase::RobotBaseDriverSimulation, public RPI::Device
	{
	public:
		static RobotBaseSim* createDevice(std::string name, RPI::parameter_t parameters);

		RobotBaseSim(std::string name, RPI::parameter_t parameters);
		virtual ~RobotBaseSim();

		std::set<std::string> getMutableParameters() const;
		void updateParameters();
		void setEStop(bool value);
		RPI::DeviceState getDeviceState() const;

		int getBaseError();
		int getCartesianPositionDeviceError();

		int checkBaseVelocity(KDL::Twist velocity);

		std::string getCartesianPositionResourceName();
		std::string getRobotBaseResourceName();


	private:
		bool estop;
	};

} /* namespace robotbase */
#endif /* ROBOTBASESIMULATION_HPP_ */
