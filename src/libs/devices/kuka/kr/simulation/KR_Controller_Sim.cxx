/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "KR_Controller_Sim.hpp"

namespace kuka_kr
{
	KR_Controller_Sim::KR_Controller_Sim(std::string name) :
			KR_Controller(), RobotArmDriverSimulation(kr_sim_jointcount), Device(name, RPI::parameter_t())
	{
		this->setJointPosition(0, 0, 0);
		this->setJointPosition(1, -KDL::PI / 2, 0);
		this->setJointPosition(2, KDL::PI / 2, 0);
		this->setJointPosition(3, 0, 0);
		this->setJointPosition(4, 0, 0);
		this->setJointPosition(5, 0, 0);
		for (int i = 6; i < kr_sim_jointcount; ++i)
			this->setJointPosition(i, 0, 0);

		isEstop = false;

		max_vel.resize(kr_sim_jointcount);
		max_acc.resize(kr_sim_jointcount);
		minj.resize(kr_sim_jointcount);
		maxj.resize(kr_sim_jointcount);
		for (int i = 0; i < kr_sim_jointcount; ++i)
		{
			maxj[i] = max_vel[i] = max_acc[i] = std::numeric_limits<double>::max();
			minj[i] = std::numeric_limits<double>::min();
		}
	}

	KR_Controller_Sim::~KR_Controller_Sim()
	{

	}

	bool KR_Controller_Sim::getPower() const
	{
		return true;
	}

	// Interface Device
	void KR_Controller_Sim::updateParameters()
	{

	}

	std::set<std::string> KR_Controller_Sim::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void KR_Controller_Sim::setEStop(bool estop)
	{
		this->isEstop = estop;
	}
	RPI::DeviceState KR_Controller_Sim::getDeviceState() const
	{
		return isEstop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL;
	}

	int KR_Controller_Sim::getJointCount() const
	{
		return kr_sim_jointcount;
	}

	robotarm::JointPositionError KR_Controller_Sim::checkJointPosition(int joint, double position)
	{
		if (position != position)
			return robotarm::JP_INVALID;

		if (joint < 0 || joint >= kr_sim_jointcount)
			return robotarm::JP_INVALID;

		if (joint >= minj.size() || joint >= maxj.size())
			return robotarm::JP_OK;

		if (position < minj[joint] || position > maxj[joint])
			return robotarm::JP_OUTOFRANGE;

		return robotarm::JP_OK;
	}

	void KR_Controller_Sim::setJointConfig(int axis, double _max_vel, double _max_acc, double minj, double maxj)
	{
		if (axis < 0 || axis >= kr_sim_jointcount)
			return;

		this->max_vel[axis] = _max_vel;
		this->max_acc[axis] = _max_acc;
		this->minj[axis] = minj;
		this->maxj[axis] = maxj;
	}

	void KR_Controller_Sim::setInitialToolConfig(double mass, KDL::Vector com)
	{
		// Do nothing in simulation mode
	}

} /* namespace kuka_kr */
