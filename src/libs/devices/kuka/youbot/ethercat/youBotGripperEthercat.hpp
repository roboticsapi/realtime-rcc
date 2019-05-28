/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef KUKA_YOUBOTGRIPPER_EC_HPP_
#define KUKA_YOUBOTGRIPPER_EC_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

#include <rcc/Module.hpp>
#include <rcc/Device.hpp>
#include <rcc/Server/HTTPServer.hpp>
#include <rcc/Registry.hpp>

#include "../device/youBotGripper.hpp"
#include "youBotArmEthercat.hpp"


namespace kuka_youbot
{
	// Device name
	const std::string dev_kuka_youbot_gripper_ec = "kuka_youbot_gripper_ec";

	enum GripperState
	{
		Gripper_Initialize, Gripper_Wait, Gripper_Move
	};

	/**
	 * This class implements a youBot gripper controller interfaced by the youBot API
	 */
	class youBotGripperEthercat:
			public youBotGripper,
			public RTT::TaskContext,
			public RPI::Device
	{
	public:
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		static youBotGripperEthercat* createDevice(std::string name, RPI::parameter_t parameters);
		virtual ~youBotGripperEthercat();

		void updateParameters();
		std::set<std::string> getMutableParameters() const;

		void setEStop(bool estop);
		RPI::DeviceState getDeviceState() const;

		bool setGripperPosition(float value);
		float getGripperPosition();
		bool gripperBusy();

	private:
		RPI::DeviceInstanceT<youBotArmEthercat> arm;

		youBotGripperEthercat(std::string name, RPI::parameter_t parameters);

		// Gripper
		float gripperGoal;
		GripperState gripperState;

		int maxEncoderValueGripper;
		double maxGripperDistance;
		double gripperOffset;

		bool checkGripperPositionInRange(const float newPosition) const;

	};
}

#endif /* KUKA_YOUBOTGRIPPER_EC_HPP_ */
