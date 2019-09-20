/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <sstream>
#include "RobotArmDriverSimulation.hpp"

namespace robotarm
{

	RobotArmDriverSimulation::RobotArmDriverSimulation(unsigned int jointcount) :
			jointcount(jointcount)
	{
		jointpos = new double[jointcount];
		jointvel = new double[jointcount];
		jointacc = new double[jointcount];
		lastticks = new RTT::os::TimeService::nsecs[jointcount];
		dumpPos = new RPI::RoundRobinLog<double>[jointcount];
		dumpVel = new RPI::RoundRobinLog<double>[jointcount];

		for (int i = 0; i < jointcount; ++i)
		{
			std::stringstream joint; joint<< "Joint " << i;
			dumpPos[i].setName(joint.str() + "|Position|RPI");
			dumpVel[i].setName(joint.str() + "|Velocity|RPI");

			jointpos[i] = 0;
			jointvel[i] = 0;
			jointacc[i] = 0;
			lastticks[i] = 0;
		}
	}

	RobotArmDriverSimulation::~RobotArmDriverSimulation()
	{
		delete[] jointpos; jointpos = 0;
		delete[] jointvel; jointvel = 0;
		delete[] jointacc; jointacc = 0;
		delete[] lastticks; lastticks = 0;
		delete[] dumpPos; dumpPos = 0;
		delete[] dumpVel; dumpVel = 0;
	}

	int RobotArmDriverSimulation::getJointError(int joint)
	{
		// 2 meaning "DrivesNotEnabled"
		return getPower() ? 0 : 2;
	}

	void RobotArmDriverSimulation::setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time)
	{
		if (joint < 0 || joint >= getJointCount())
			return;

		// calculate velocity and acceleration
		double dt = (time - lastticks[joint]) / 1000000000.0;
		double jointv = (position - jointpos[joint]) / dt;
		double jointa = (jointv - jointvel[joint]) / dt;

		jointvel[joint] = jointv;
		jointacc[joint] = jointa;

		lastticks[joint] = time;
		jointpos[joint] = position;

		dumpPos[joint].put(position, (time));
		dumpVel[joint].put(jointv, (time));
	}

	double RobotArmDriverSimulation::getMeasuredJointPosition(int joint)
	{
		if (joint < 0 || joint >= getJointCount())
			return 0;
		return jointpos[joint];
	}
	double RobotArmDriverSimulation::getCommandedJointPosition(int joint)
	{
		if (joint < 0 || joint >= getJointCount())
			return 0;
		return jointpos[joint];
	}

	double RobotArmDriverSimulation::getMeasuredJointVelocity(int joint)
	{
		if (joint < 0 || joint >= getJointCount())
			return 0;

		RTT::os::TimeService::nsecs time = RTT::os::TimeService::Instance()->getNSecs();
		double dt = (time - lastticks[joint]) / 1000000000.0;
		if(dt > 0.1) jointvel[joint] = 0;

		return jointvel[joint];
	}

	double RobotArmDriverSimulation::getCommandedJointVelocity(int joint)
	{
		if (joint < 0 || joint >= getJointCount())
			return 0;

		RTT::os::TimeService::nsecs time = RTT::os::TimeService::Instance()->getNSecs();
		double dt = (time - lastticks[joint]) / 1000000000.0;
		if(dt > 0.1) jointvel[joint] = 0;

		return jointvel[joint];
	}

	double RobotArmDriverSimulation::getMeasuredJointAcceleration(int joint)
	{
		if (joint < 0 || joint >= getJointCount())
			return 0;
		return jointacc[joint];
	}
	double RobotArmDriverSimulation::getCommandedJointAcceleration(int joint)
	{
		if (joint < 0 || joint >= getJointCount())
			return 0;
		return jointacc[joint];
	}

	void RobotArmDriverSimulation::setToolCOM(KDL::Vector com, int axis)
	{
		// Do nothing in simulation
	}

	void RobotArmDriverSimulation::setToolMOI(KDL::Vector moi, int axis)
	{
		// Do nothing in simulation
	}

	void RobotArmDriverSimulation::setToolMass(double mass, int axis)
	{
		// Do nothing in simulation
	}

	bool RobotArmDriverSimulation::getToolFinished(int axis) const
	{
		// Tool actions always succeed in simulation
		return true;
	}

	int RobotArmDriverSimulation::getToolError(int axis) const
	{
		// Tool actions always succeed in simulation
		return 0;
	}

	std::list<RPI::CrashDumper*> RobotArmDriverSimulation::getCrashDumpers() const
	{
		std::list<RPI::CrashDumper*> ret;
		for(int i=0; i<jointcount; i++) {
			ret.push_back(&dumpPos[i]);
			ret.push_back(&dumpVel[i]);
		}
		return ret;
	}

} /* namespace robotarm */
