/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "KR_ExtAxis_Device.hpp"
#include <rcc/Module.hpp>

#include <rtt/Logger.hpp>

namespace kuka_kr
{
	const std::string KR_ExtAxis_Device::ext_devicename = "kuka_kr_extaxis";

	KR_ExtAxis_Device::KR_ExtAxis_Device(std::string name, RPI::parameter_t parameters) :
			RobotArmInterface(), Device(name, parameters)
	{
		this->name = name;

		std::vector<double> minj = RPI::Module::parseString<double>(getParameter("min_joint"));
		std::vector<double> maxj = RPI::Module::parseString<double>(getParameter("max_joint"));

		std::vector<double> max_vel = RPI::Module::parseString<double>(getParameter("max_vel"));
		std::vector<double> max_acc = RPI::Module::parseString<double>(getParameter("max_acc"));

		initialized = true;

		std::string dev_name = getParameter("kr_arm", "krrsi0");
		if (!kr_arm.fetchInstance(dev_name))
		{
			RTT::log() << "KR ExtAxis Device could not be created, KR " << dev_name << " not found!" << RTT::endlog();
			initialized = false;
		} else
		{
			kr_ext_axiscount = getParameterT<int>("axiscount", 1);
			if (kr_ext_axiscount < 1 || kr_ext_axiscount > kr_arm->getController()->getJointCount())
			{
				RTT::log() << "KR ExtAxis invalid axis count " << kr_ext_axiscount << RTT::endlog();
				initialized = false;
			}

			kr_first_axis = getParameterT<int>("first_axis", 6);
			if (kr_first_axis < 0 || (kr_first_axis + kr_ext_axiscount) > kr_arm->getController()->getJointCount())
			{
				RTT::log() << "KR ExtAxis invalid first axis " << kr_first_axis << RTT::endlog();
				initialized = false;
			}

			for (int i = 0; i < kr_ext_axiscount; ++i)
			{
				kr_arm->getController()->setJointConfig(kr_first_axis + i, max_vel[i], max_acc[i], minj[i], maxj[i]);
			}

		}
	}

	KR_ExtAxis_Device::~KR_ExtAxis_Device()
	{

	}

	// Interface Device
	void KR_ExtAxis_Device::updateParameters()
	{

	}

	std::set<std::string> KR_ExtAxis_Device::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	KR_ExtAxis_Device* KR_ExtAxis_Device::createDevice(std::string name, RPI::parameter_t parameters)
	{
		KR_ExtAxis_Device* kr_rsi = new KR_ExtAxis_Device(name, parameters);
		return kr_rsi;
	}

	void KR_ExtAxis_Device::setEStop(bool estop)
	{
		if (initialized)
			kr_arm->getController()->setEStop(estop);
	}

	RPI::DeviceState KR_ExtAxis_Device::getDeviceState() const
	{
		if (initialized)
			return kr_arm->getController()->getDeviceState();
		else
			return RPI::DeviceState::OFFLINE;
	}

	// Interface RobotArm
	int KR_ExtAxis_Device::getJointCount() const
	{
		return kr_ext_axiscount;
	}

	int KR_ExtAxis_Device::getJointError(int joint)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->getJointError(kr_first_axis + joint);
		return 0;
	}

	robotarm::JointPositionError KR_ExtAxis_Device::checkJointPosition(int joint, double position)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->checkJointPosition(kr_first_axis + joint, position);
		return robotarm::JointPositionError::JP_INVALID;
	}

	void KR_ExtAxis_Device::setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			kr_arm->getController()->setJointPosition(kr_first_axis + joint, position, time);
	}

	double KR_ExtAxis_Device::getMeasuredJointPosition(int joint)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->getMeasuredJointPosition(kr_first_axis + joint);
		return 0;
	}

	double KR_ExtAxis_Device::getCommandedJointPosition(int joint)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->getCommandedJointPosition(kr_first_axis + joint);
		return 0;
	}

	double KR_ExtAxis_Device::getMeasuredJointVelocity(int joint)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->getMeasuredJointVelocity(kr_first_axis + joint);
		return 0;
	}

	double KR_ExtAxis_Device::getCommandedJointVelocity(int joint)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->getCommandedJointVelocity(kr_first_axis + joint);
		return 0;
	}

	double KR_ExtAxis_Device::getMeasuredJointAcceleration(int joint)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->getMeasuredJointAcceleration(kr_first_axis + joint);
		return 0;
	}

	double KR_ExtAxis_Device::getCommandedJointAcceleration(int joint)
	{
		if (initialized && joint >= 0 && joint < kr_ext_axiscount)
			return kr_arm->getController()->getCommandedJointAcceleration(kr_first_axis + joint);
		return 0;
	}

	/*following functions aren't necessary */
	void KR_ExtAxis_Device::setToolCOM(KDL::Vector com, int axis)
	{

	}
	void KR_ExtAxis_Device::setToolMOI(KDL::Vector moi, int axis)
	{

	}
	void KR_ExtAxis_Device::setToolMass(double mass, int axis)
	{

	}
	bool KR_ExtAxis_Device::getToolFinished(int axis) const
	{
		return true;
	}
	int KR_ExtAxis_Device::getToolError(int axis) const
	{
		return 0;
	}

} /* namespace kuka_kr */
