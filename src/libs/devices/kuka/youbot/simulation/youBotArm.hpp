/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef KUKA_YOUBOTARM_SIM_HPP_
#define KUKA_YOUBOTARM_SIM_HPP_

#include <rcc/Device.hpp>
#include <libs/framework/robotarm/simulation/RobotArmDriverSimulation.hpp>
#include "../kinematics/yb_Kin.hpp"
//#include "../kinematics/InvKinABCZ.hpp"

#include "../device/youBotArm.hpp"

namespace kuka_youbot
{

	const std::string dev_kuka_youbot_arm_sim = "kuka_youbot_arm_sim";

	class youBotArmSimulation:
			public youBotArm,
			public robotarm::RobotArmDriverSimulation,
			public RPI::Device
	{
	public:
		youBotArmSimulation(std::string name, RPI::parameter_t parameters);
		virtual ~youBotArmSimulation();

		static youBotArmSimulation* createDevice(std::string name, RPI::parameter_t parameters);

		virtual bool getPower() const;

		// Interface Device
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		void setEStop(bool estop);
		RPI::DeviceState getDeviceState() const;

		// Interface ArmKinematics
		KDL::Frame Kin(const RPI::Array<double>& joints);
		void InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position, RPI::Array<double>& values);

		// Interface RobotArm
		virtual int getJointCount() const;
		virtual robotarm::JointPositionError checkJointPosition(int joint, double position);

		// Interface youBotArm
		virtual void getJointFirmware(std::string& j1, std::string& j2, std::string& j3, std::string& j4,
				std::string& j5);
		virtual void getJointParameter(int& j1, int& j2, int& j3, int& j4, int& j5, int ParamNumber);
		virtual bool getJointError(int axis, int bit);
		virtual bool setGripperPosition(float value);
		virtual bool gripperBusy();

		virtual void setControllerIndex(int controllerIndex);
		virtual bool checkControllerIndex(int controllerIndex);
		virtual void setControllerJointImpedanceParameters(int joint, float stiffness,float damping,float maxTorque);
		virtual bool checkControllerJointImpedanceParameters(int joint, float stiffness, float damping,float maxTorque);
		virtual void setControllerPositionGainConstant(int joint, float gain);
		virtual void setControllerJointImpedanceAddionalTorque(int joint, float torque);
		virtual bool checkControllerJointImpedanceAdditionalTorque(int joint, float torque);

	private:
		bool isEStop;

		double maxGripperDistance;
		double gripperOffset;
		bool checkGripperPositionInRange(const float newPosition) const;

		yb_kin kin;
//		InvKinABCZ invkin;

	};

}
#endif /* KUKA_YOUBOTARM_SIM_HPP_ */
