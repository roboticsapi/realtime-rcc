/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "lwa_simulation.hpp"

namespace schunk_lwa
{
	const std::string LWA_Simulation::lwa_devicename = "schunk_lwa_sim";

	LWA_Simulation::LWA_Simulation(std::string name, RPI::parameter_t parameters) :
			RobotArmDriverSimulation(lwa_jointcount), RPI::Device(name, parameters)
	{
		this->name = name;
		this->isEStop = false;

		// Kinematics
		dh_param.dh_d = RPI::Module::parseString<double>(getParameter("dh_d"));
		dh_param.dh_t = RPI::Module::parseString<double>(getParameter("dh_t"));
		dh_param.dh_a = RPI::Module::parseString<double>(getParameter("dh_a"));
		dh_param.dh_al = RPI::Module::parseString<double>(getParameter("dh_al"));

		minj = RPI::Module::parseString<double>(getParameter("min_joint"));
		maxj = RPI::Module::parseString<double>(getParameter("max_joint"));

		KDL::JntArray q_min(minj.size()), q_max(maxj.size());

		for (int i = 0; i < minj.size(); ++i)
			q_min(i) = minj[i];
		for (int i = 0; i < maxj.size(); ++i)
			q_max(i) = maxj[i];

		kincalc = new Schunk_KinGeo_Calculation(dh_param, q_min, q_max);

		/*this->setJointPosition(0, 0, 0);
		this->setJointPosition(1, -KDL::PI / 2, 0);
		this->setJointPosition(2, KDL::PI / 2, 0);
		this->setJointPosition(3, 0, 0);
		this->setJointPosition(4, 0, 0);
		this->setJointPosition(5, 0, 0);*/

	}

	LWA_Simulation::~LWA_Simulation()
	{

	}

	LWA_Simulation* LWA_Simulation::createDevice(std::string name, RPI::parameter_t parameters)
	{
		return new LWA_Simulation(name, parameters);
	}

	bool LWA_Simulation::getPower() const
	{
		return !isEStop;
	}

	// Interface Device
	void LWA_Simulation::updateParameters()
	{

	}

	std::set<std::string> LWA_Simulation::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void LWA_Simulation::setEStop(bool estop)
	{
		isEStop = estop;
	}

	RPI::DeviceState LWA_Simulation::getDeviceState() const
	{
		return getPower() ? RPI::DeviceState::OPERATIONAL : RPI::DeviceState::SAFE_OPERATIONAL;
	}

	// Interface ArmKinematics
	KDL::Frame LWA_Simulation::Kin(const RPI::Array<double>& joints)
	{
		return kincalc->Kin(joints);
	}

	void LWA_Simulation::InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position,
			RPI::Array<double>& values)
	{
		// We need exactly 6 axes
		if (hintJoints.getSize() != lwa_jointcount || values.getSize() != lwa_jointcount)
			return;

		kincalc->InvKin(hintJoints, position, values);
	}

	// Interface RobotArm
	int LWA_Simulation::getJointCount() const
	{
		return lwa_jointcount;
	}

	robotarm::JointPositionError LWA_Simulation::checkJointPosition(int joint, double position)
	{
		if (position != position)
			return robotarm::JP_INVALID;

		if (joint < 0 || joint >= lwa_jointcount)
			return robotarm::JP_INVALID;

		if (joint >= minj.size() || joint >= maxj.size())
			return robotarm::JP_OK;

		if (position < minj[joint] || position > maxj[joint])
			return robotarm::JP_OUTOFRANGE;

		return robotarm::JP_OK;
	}

} /* namespace kuka_kr */
