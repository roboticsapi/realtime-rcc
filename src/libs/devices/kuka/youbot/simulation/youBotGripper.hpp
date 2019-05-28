/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef KUKA_YOUBOTGRIPPER_SIM_HPP_
#define KUKA_YOUBOTGRIPPER_SIM_HPP_

#include <rcc/Device.hpp>

#include "../device/youBotGripper.hpp"

namespace kuka_youbot
{

	const std::string dev_kuka_youbot_gripper_sim = "kuka_youbot_gripper_sim";

	class youBotGripperSimulation:
			public youBotGripper,
			public RPI::Device
	{
	public:
		youBotGripperSimulation(std::string name, RPI::parameter_t parameters);
		virtual ~youBotGripperSimulation();

		static youBotGripperSimulation* createDevice(std::string name, RPI::parameter_t parameters);

		// Interface Device
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		void setEStop(bool estop);
		RPI::DeviceState getDeviceState() const;

		// Interface youBotGripper
		virtual bool setGripperPosition(float value);
		virtual float getGripperPosition();
		virtual bool gripperBusy();


	private:
		bool isEStop;

		double maxGripperDistance;
		double gripperOffset;
		double gripperPosition;
		bool checkGripperPositionInRange(const float newPosition) const;

	};

}
#endif /* KUKA_YOUBOTGRIPPER_SIM_HPP_ */
