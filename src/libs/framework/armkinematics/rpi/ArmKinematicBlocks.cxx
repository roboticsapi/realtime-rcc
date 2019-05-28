/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "ArmKinematicBlocks.hpp"
#include <rtt/Logger.hpp>

namespace armkinematics
{

	InvKin::InvKin(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inFrame("inFrame", this), inHintJoints("inHintJoints", this), outJoints(
					"outJoints", this), propRobot("Robot",
					"Robot device", "")
	{
		setDescription("Inverse kinematics module for robot arms");
		this->ports()->addPort(&inHintJoints, "Hint joints");
		this->ports()->addPort(&inFrame, "Destination frame");
		this->ports()->addPort(&outJoints, "Result angle array for all joints");

		this->properties()->addProperty(&propRobot);
	}

	InvKin::~InvKin()
	{

	}

	bool InvKin::configureHook()
	{
		if (!inFrame.connected())
			return false;
		if (!inHintJoints.connected())
			return false;

		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "Robot " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}

		joints.resize(devins->getJointCount());

		return true;
	}

	bool InvKin::startHook()
	{
		return true;
	}

	void InvKin::updateHook()
	{
		if (active())
		{
			devins.getDevice()->InvKin(inHintJoints.Get(), inFrame.Get(), joints);

			outJoints.Set(joints);
		}
	}

	void InvKin::stopHook()
	{
	}

	void InvKin::cleanupHook()
	{
	}

	Kin::Kin(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inJoints("inJoints", this), outFrame("outFrame", this), propRobot("Robot",
					"Robot device", "")
	{
		setDescription("Direct kinematics module for robot arms");
		this->ports()->addPort(&inJoints, "Value of all joints");
		this->ports()->addPort(&outFrame, "Resulting frame");

		this->properties()->addProperty(&propRobot);
	}

	Kin::~Kin()
	{

	}

	bool Kin::configureHook()
	{
		if (!inJoints.connected())
			return false;

		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "Robot " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}

		return true;
	}

	bool Kin::startHook()
	{
		return true;
	}

	void Kin::updateHook()
	{
		if (active())
		{
			outFrame.Set(devins.getDevice()->Kin(inJoints.Get()));
		}
	}

	void Kin::stopHook()
	{
		// Your stop code after last updateHook()
	}

	void Kin::cleanupHook()
	{
		// Your configuration cleanup code
	}

} /* namespace armkinematics */
