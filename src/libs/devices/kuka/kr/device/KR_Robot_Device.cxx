/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "KR_Robot_Device.hpp"

namespace kuka_kr
{

	KR_Robot_Device::KR_Robot_Device(std::string name, RPI::parameter_t parameters, KR_Controller* controller) :
			RobotArmInterface(), Device(name, parameters)
	{
		this->controller = controller;
		this->name = name;

		kincalc = 0; // child classes initialize kinematics

		minj = RPI::Module::parseString<double>(getParameter("min_joint"));
		maxj = RPI::Module::parseString<double>(getParameter("max_joint"));

		max_vel = RPI::Module::parseString<double>(getParameter("max_vel"));
		max_acc = RPI::Module::parseString<double>(getParameter("max_acc"));

		// Resize arrays
		minj.resize(kr_jointcount);
		maxj.resize(kr_jointcount);
		max_vel.resize(kr_jointcount);
		max_acc.resize(kr_jointcount);

		// Update RSI driver with velocity and acceleration data
		// required for automatic braking when RPI net terminates prematurely (e.g.
		// on emergency stop)
		for (int i = 0; i < kr_jointcount; ++i)
		{
			controller->setJointConfig(i, max_vel[i], max_acc[i], minj[i], maxj[i]);
		}

		// Tool configuration
		RPI::TypeKit* vectortk = RPI::TypeKits::getInstance()->getTypeKit<KDL::Vector>();
		KDL::Vector com;
		vectortk->fromString(&com, getParameter("com"));

		// Set mass to 1 kg default
		controller->setInitialToolConfig(getParameterT<double>("mass", 1), com);
	}

	KR_Robot_Device::~KR_Robot_Device()
	{
	}

	// Interface Device
	void KR_Robot_Device::updateParameters()
	{

	}

	std::set<std::string> KR_Robot_Device::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void KR_Robot_Device::setEStop(bool estop)
	{
		controller->setEStop(estop);
	}

	// Interface ArmKinematics
	KDL::Frame KR_Robot_Device::Kin(const RPI::Array<double>& joints)
	{
		return kincalc->Kin(joints);
	}

	void KR_Robot_Device::InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position,
			RPI::Array<double>& values)
	{
		// We need exactly 6 axes
		if (hintJoints.getSize() != kr_jointcount || values.getSize() != kr_jointcount)
			return;

		kincalc->InvKin(hintJoints, position, values);
	}

	// Interface RobotArm
	int KR_Robot_Device::getJointCount() const
	{
		return kr_jointcount;
	}

	robotarm::JointPositionError KR_Robot_Device::checkJointPosition(int joint, double position)
	{
		return controller->checkJointPosition(joint, position);
	}

	int KR_Robot_Device::getJointError(int joint)
	{
		return controller->getJointError(joint);
	}

	double KR_Robot_Device::getMeasuredJointPosition(int joint)
	{
		if (joint >= 0 && joint < kr_jointcount)
			return controller->getMeasuredJointPosition(joint);
		return 0;
	}

	double KR_Robot_Device::getMeasuredJointVelocity(int joint)
	{
		if (joint >= 0 && joint < kr_jointcount)
			return controller->getMeasuredJointVelocity(joint);
		return 0;
	}

	double KR_Robot_Device::getMeasuredJointAcceleration(int joint)
	{
		if (joint >= 0 && joint < kr_jointcount)
			return controller->getMeasuredJointAcceleration(joint);
		return 0;
	}

	void KR_Robot_Device::setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time)
	{
		if (joint >= 0 && joint < kr_jointcount)
			controller->setJointPosition(joint, position, time);

	}

	double KR_Robot_Device::getCommandedJointPosition(int joint)
	{
		if (joint >= 0 && joint < kr_jointcount)
			return controller->getCommandedJointPosition(joint);
		return 0;
	}

	double KR_Robot_Device::getCommandedJointVelocity(int joint)
	{
		if (joint >= 0 && joint < kr_jointcount)
			return controller->getCommandedJointVelocity(joint);
		return 0;
	}

	double KR_Robot_Device::getCommandedJointAcceleration(int joint)
	{
		if (joint >= 0 && joint < kr_jointcount)
			return controller->getCommandedJointAcceleration(joint);
		return 0;
	}

	void KR_Robot_Device::setToolCOM(KDL::Vector com, int axis)
	{
		controller->setToolCOM(com, axis);
	}

	void KR_Robot_Device::setToolMOI(KDL::Vector moi, int axis)
	{
		controller->setToolMOI(moi, axis);
	}

	void KR_Robot_Device::setToolMass(double mass, int axis)
	{
		controller->setToolMass(mass, axis);
	}

	bool KR_Robot_Device::getToolFinished(int axis) const
	{
		return controller->getToolFinished(axis);
	}

	int KR_Robot_Device::getToolError(int axis) const
	{
		return controller->getToolError(axis);
	}

	RPI::DeviceState KR_Robot_Device::getDeviceState() const
	{
		return controller->getDeviceState();
	}

	KR_Controller* KR_Robot_Device::getController() const
	{
		return controller;
	}

} /* namespace kuka_kr */
