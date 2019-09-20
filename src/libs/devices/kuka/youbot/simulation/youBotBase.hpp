/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef KUKA_YOUBOTBASE_SIM_HPP_
#define KUKA_YOUBOTBASE_SIM_HPP_


#include <rcc/Device.hpp>
#include <libs/framework/robotbase/simulation/RobotBaseDriverSimulation.hpp>
#include <libs/framework/cartesianposition/device/CartesianPositionDevice.hpp>
#include "../device/youBotBase.hpp"
#include "../kinematics/base_Kin.hpp"

namespace kuka_youbot
{

	const std::string dev_kuka_youbot_base_sim = "kuka_youbot_base_sim";

	class youBotBaseSimulation :
			youBotBase,
			public virtual robotbase::RobotBaseDriverSimulation,
			public RPI::Device
	{
	public:
		static youBotBaseSimulation* createDevice(std::string name, RPI::parameter_t parameters);

		youBotBaseSimulation(std::string name, RPI::parameter_t parameters);
		virtual ~youBotBaseSimulation();

		std::set<std::string> getMutableParameters() const;
		void updateParameters();
		void setEStop(bool value);
		RPI::DeviceState getDeviceState() const;

		// RobotBase
		int getBaseError();
		int checkBaseVelocity(KDL::Twist velocity);

		int getWheelCount() const;
		double getWheelPosition(int wheel) const;
		double getWheelVelocity(int wheel) const;
		std::string getRobotBaseResourceName();

		void updateHook();

		// CartesianPosition
		std::string getCartesianPositionResourceName();
		int getCartesianPositionDeviceError();

	private:
		bool estop;
		base_Kin kin;
		double wheels[4], wheelvel[4];
	};

}
#endif /* KUKA_YOUBOTBASE_SIM_HPP_ */
