/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotArm.hpp"

namespace kuka_youbot
{

	youBotArmSimulation::youBotArmSimulation(std::string name, RPI::parameter_t parameters) :
			RPI::Device(name, parameters), robotarm::RobotArmDriverSimulation(5), isEStop(false),
			maxGripperDistance(0.023), gripperOffset(0)
	{
		for(const auto& dumper: RobotArmDriverSimulation::getCrashDumpers())
			addCrashDumper(dumper);
	}
	youBotArmSimulation::~youBotArmSimulation()
	{

	}

	youBotArmSimulation* youBotArmSimulation::createDevice(std::string name, RPI::parameter_t parameters)
	{
		youBotArmSimulation* ret = new youBotArmSimulation(name, parameters);
		return ret;
	}

	void youBotArmSimulation::setEStop(bool estop)
	{
		isEStop = estop;
	}

	bool youBotArmSimulation::getPower() const
	{
		return !isEStop;
	}

	void youBotArmSimulation::updateParameters()
	{

	}

	std::set<std::string> youBotArmSimulation::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	RPI::DeviceState youBotArmSimulation::getDeviceState() const
	{
		return isEStop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL;
	}

	void youBotArmSimulation::getJointFirmware(std::string& j1, std::string& j2, std::string& j3, std::string& j4,
			std::string& j5)
	{
		j1 = j2 = j3 = j4 = j5 = "simulated";
	}

	void youBotArmSimulation::getJointParameter(int& j1, int& j2, int& j3, int& j4, int& j5, int ParamNumber)
	{
		j1 = j2 = j3 = j4 = j5 = 0;
	}

	bool youBotArmSimulation::getJointError(int axis, int bit)
	{
		return 0;
	}

	bool youBotArmSimulation::setGripperPosition(float value)
	{
		return checkGripperPositionInRange(value);
	}

	bool youBotArmSimulation::gripperBusy()
	{
		return false;
	}

	bool youBotArmSimulation::checkGripperPositionInRange(const float newPosition) const
	{
		float newPos_roundedUp = (ceilf(newPosition * 1000) / 1000);
		float upperRange = (ceilf((maxGripperDistance + gripperOffset) * 1000) / 1000);

		if (newPosition < 0 || newPos_roundedUp > upperRange)
		{
			return false;
		}
		return true;
	}

	// Interface RobotArm
	int youBotArmSimulation::getJointCount() const
	{
		return 5;
	}

	robotarm::JointPositionError youBotArmSimulation::checkJointPosition(int joint, double position)
	{
		return (position != position) ? robotarm::JP_INVALID : robotarm::JP_OK;
	}

	// Interface ArmKinematics
	KDL::Frame youBotArmSimulation::Kin(const RPI::Array<double>& joints)
	{
		KDL::Frame pos;
		kin.ybKin(joints.get(0), joints.get(1), joints.get(2), joints.get(3), joints.get(4), pos);
		return pos;
	}

	void youBotArmSimulation::InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position,
			RPI::Array<double>& values)
	{
		if (hintJoints.getSize() != 5 || values.getSize() != 5)
			return;
		double j[5];
		for (int i = 0; i < 5; i++)
		{
			j[i] = hintJoints.get(i);
		}
//		invkin.invKin(position, j[0], j[1], j[2], j[3], j[4]);
		kin.ybInvKin(position, j[0], j[1], j[2], j[3], j[4]);
		for (int i = 0; i < 5; i++)
		{
			values[i] = j[i];
		}
	}


	void youBotArmSimulation::setControllerIndex(int controllerIndex)
	{

	}

	bool youBotArmSimulation::checkControllerIndex(int controllerIndex)
	{
		return true;
	}

	void youBotArmSimulation::setControllerJointImpedanceParameters(int joint, float stiffness, float damping, float maxTorque)
	{

	}

	bool youBotArmSimulation::checkControllerJointImpedanceParameters(int joint, float stiffness, float damping, float maxTorque)
	{
		return true;
	}

	void youBotArmSimulation::setControllerPositionGainConstant(int joint, float gain)
	{

	}

	void youBotArmSimulation::setControllerJointImpedanceAddionalTorque(int joint, float torque)
	{

	}

	bool youBotArmSimulation::checkControllerJointImpedanceAdditionalTorque(int joint, float torque)
	{
		return true;
	}

}

