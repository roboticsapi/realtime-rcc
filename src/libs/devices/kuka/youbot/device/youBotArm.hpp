/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTARM_HPP_
#define YOUBOTARM_HPP_

#include <rcc/Device.hpp>
#include <rcc/DeviceFactory.hpp>
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>

namespace kuka_youbot
{

	class RTT_EXPORT youBotArm: public virtual robotarm::RobotArmInterface, virtual public armkinematics::ArmKinematicsInterface
	{
	public:

		virtual ~youBotArm()
		{
		}

		virtual void getJointFirmware(std::string& j1, std::string& j2, std::string& j3, std::string& j4,
				std::string& j5) = 0;
		virtual void getJointParameter(int& j1, int& j2, int& j3, int& j4, int& j5, int ParamNumber) = 0;
		virtual bool getJointError(int axis, int bit) = 0;

		virtual void setControllerIndex(int controllerIndex)= 0;
		virtual bool checkControllerIndex(int controllerIndex)= 0;
		virtual void setControllerJointImpedanceParameters(int joint, float stiffness,float damping, float maxTorque)= 0;
		virtual bool checkControllerJointImpedanceParameters(int joint, float stiffness, float damping, float maxTorque)= 0;
		virtual void setControllerPositionGainConstant(int joint, float gain)= 0;
		virtual void setControllerJointImpedanceAddionalTorque(int joint, float torque)= 0;
		virtual bool checkControllerJointImpedanceAdditionalTorque(int joint, float torque)= 0;

	};
}
#endif /* YOUBOTARM_HPP_ */
