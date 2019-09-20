/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <sstream>
#include "CyclicPositionRobotArm.hpp"

namespace robotarm
{

	CyclicPositionRobotArm::CyclicPositionRobotArm(int jointCount, int smoothLength) :
			jointCount(jointCount), tickDiffCount(smoothLength)
	{
		RPIPos = new double[jointCount];
		RPIVel = new double[jointCount];
		RPIAcc = new double[jointCount];
		RPIPos1 = new double[jointCount];
		RPIPos2 = new double[jointCount];
		RPITicks = new RTT::os::TimeService::nsecs[jointCount];
		RPITicks1 = new RTT::os::TimeService::nsecs[jointCount];
		RPITicks2 = new RTT::os::TimeService::nsecs[jointCount];
		dumpRPIPos = new RPI::RoundRobinLog<double>[jointCount];
		dumpRPIVel = new RPI::RoundRobinLog<double>[jointCount];
		dumpCmdPos = new RPI::RoundRobinLog<double>[jointCount];
		dumpCmdVel = new RPI::RoundRobinLog<double>[jointCount];
		dumpCmdAge = new RPI::RoundRobinLog<double>[jointCount];
		dumpCmdSmoothAge = new RPI::RoundRobinLog<double>[jointCount];

		CPPos = new double[jointCount];
		CPVel = new double[jointCount];

		for(int i=0; i<jointCount; i++) {
			std::stringstream joint; joint << "Joint " << i;
			RPIPos[i] = 0; RPIPos1[i] = 0; RPIPos2[i] = 0; 
			RPIVel[i] = 0; RPIAcc[i] = 0;
			RPITicks[i] = 0; RPITicks1[i] = 0; RPITicks2[i] = 0;
			CPPos[i] = 0; CPVel[i] = 0;
			dumpRPIPos[i].setName(joint.str() + "|Position|RPI");
			dumpRPIVel[i].setName(joint.str() + "|Velocity|RPI");
			dumpCmdPos[i].setName(joint.str() + "|Position|CyclicPosition");
			dumpCmdVel[i].setName(joint.str() + "|Velocity|CyclicPosition");
			dumpCmdAge[i].setName(joint.str() + "|Position|RPI Age");
			dumpCmdSmoothAge[i].setName(joint.str() + "|Position|Smooth Age");
		}
		lastCPTicks = lastRealTicks = 0;

		tickDiffs = new RTT::os::TimeService::nsecs[smoothLength];
		for(int i=0; i<smoothLength; i++) {
			tickDiffs[i] = 0;
		}

		resetTimeHistory();
	}

	CyclicPositionRobotArm::~CyclicPositionRobotArm()
	{
		delete[] RPIPos; RPIPos = 0;
		delete[] RPIVel; RPIVel = 0;
		delete[] RPIAcc; RPIAcc = 0;
		delete[] RPIPos1; RPIPos1 = 0;
		delete[] RPIPos2; RPIPos2 = 0;
		delete[] RPITicks; RPITicks = 0;
		delete[] RPITicks1; RPITicks1 = 0;
		delete[] RPITicks2; RPITicks2 = 0;
		delete[] CPPos; CPPos = 0;
		delete[] CPVel; CPVel = 0;
		delete[] dumpRPIPos; dumpRPIPos = 0;
		delete[] dumpRPIVel; dumpRPIVel = 0;
		delete[] dumpCmdPos; dumpCmdPos = 0;
		delete[] dumpCmdVel; dumpCmdVel = 0;
	}

	void CyclicPositionRobotArm::setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time)
	{
		if (joint < 0 || joint >= jointCount)
			return;

		if(checkJointPosition(joint, position) != JP_OK)
			return;

		// calculate velocity and acceleration
		double dt = (time - RPITicks[joint]) / 1e9;
		double jointv = (position - RPIPos[joint]) / dt;
		double jointa = (jointv - RPIVel[joint]) / dt;

		if(fabs(jointv) > getMaximumVelocity(joint) * 4) // jump commanded - ignore vel+acc
		{
			jointv = 0;
			jointa = 0;
		}
		else if(fabs(jointa) > getMaximumAcceleration(joint)*4)
		{
			jointa = 0;
		}

		RPIVel[joint] = jointv;
		RPIAcc[joint] = jointa;

		RPITicks[joint] = time;
		RPIPos[joint] = position;

		dumpRPIPos[joint].put(position, time);
		dumpRPIVel[joint].put(jointv, time);
	}

	void CyclicPositionRobotArm::setJointPositionStatic(int joint, double position)
	{
		if (joint < 0 || joint >= jointCount)
			return;
		if(checkJointPosition(joint, position) != JP_OK)
			return;
		CPPos[joint] = RPIPos[joint] = position;
		RPIVel[joint] = 0;
		RPIAcc[joint] = 0;
	}

	double CyclicPositionRobotArm::getCommandedJointPosition(int joint)
	{
		return RPIPos[joint];
	}

	double CyclicPositionRobotArm::getCommandedJointVelocity(int joint)
	{
		return RPIVel[joint];
	}

	double CyclicPositionRobotArm::getCommandedJointAcceleration(int joint)
	{
		return RPIAcc[joint];
	}

	void CyclicPositionRobotArm::resetTimeHistory()
	{
		tickDiffPos = -1;
		tickDiffSum = 0;
	}

	void CyclicPositionRobotArm::getValuesToCommand(double* position, double* velocity)
	{
		getValuesToCommand(RTT::os::TimeService::Instance()->getNSecs(), position, velocity);
	}

	void CyclicPositionRobotArm::getValuesToCommand(double* position, double* velocity, double& cycletime)
	{
		getValuesToCommand(RTT::os::TimeService::Instance()->getNSecs(), position, velocity, cycletime);
	}

	void CyclicPositionRobotArm::getValuesToCommand(RTT::os::TimeService::nsecs realTicks, double* position, double* velocity)
	{
		double cycleTime;
		getValuesToCommand(realTicks, position, velocity, cycleTime);
	}

	void CyclicPositionRobotArm::getValuesToCommand(RTT::os::TimeService::nsecs realTicks, double* position, double* velocity, double& cycletime)
	{
		RTT::os::TimeService::nsecs CPTicks;
		if(tickDiffCount > 0) {
			if(tickDiffPos > -1) {
				RTT::os::TimeService::nsecs tickDiff = realTicks - lastRealTicks;
				if(tickDiffPos >= tickDiffCount)
					tickDiffSum -= tickDiffs[tickDiffPos % tickDiffCount];
				tickDiffs[tickDiffPos % tickDiffCount] = tickDiff;
				tickDiffSum += tickDiff;
				if(tickDiffPos >= tickDiffCount)
					CPTicks = lastCPTicks + tickDiffSum / tickDiffCount;
				else
					CPTicks = realTicks;
			} else {
				CPTicks = realTicks;
			}
		} else {
			CPTicks = realTicks;
		}
		cycletime = (CPTicks - lastCPTicks) / 1e9;
		lastRealTicks = realTicks;
		lastCPTicks = CPTicks;
		tickDiffPos++;

		for (int i = 0; i < jointCount; i++)
		{

			double lastCPPos = CPPos[i];
			
			if (RPITicks1[i] < RPITicks[i]) {
				RPITicks2[i] = RPITicks1[i];
				RPIPos2[i] = RPIPos1[i];
				RPITicks1[i] = RPITicks[i];
				RPIPos1[i] = RPIPos[i];
			}
			
			if (fabs((RPITicks1[i] - CPTicks) / 1e9) < cycletime * 10)
			{
				// If we have values from RPI that are newer 10 cycles old,
				// use values from RPI net
				if(RPITicks1[i] != RPITicks2[i] && tickDiffCount > 0)
				{
					// we want smoothing
					CPPos[i] = RPIPos2[i]
							+ (RPIPos1[i] - RPIPos2[i]) * (CPTicks - RPITicks2[i])
									/ (RPITicks1[i] - RPITicks2[i]);

					// never return illegal values
					if(checkJointPosition(i, CPPos[i]) != JP_OK)
						CPPos[i] = RPIPos[i];
				}
				else
				{
					// just send the data from RPI
					 CPPos[i] = RPIPos[i];
				}
			}
			else // no values have been provided by RPI net, brake to standstill
			{
				double delta = getMaximumAcceleration(i) * cycletime * cycletime;

				if (fabs(CPVel[i]*cycletime) > delta)
				{
					double pos = lastCPPos
					       + CPVel[i] * cycletime
					            - delta * ((CPVel[i] > 0) ? 1 : -1);
					setJointPositionStatic(i, pos);
				}
				else
				{
					setJointPositionStatic(i, CPPos[i]);
				}
				RPIVel[i] = 0;
			}
			CPVel[i] = (CPPos[i] - lastCPPos) / cycletime;

			if(position != 0) position[i] = CPPos[i];
			if(velocity != 0) velocity[i] = CPVel[i];

			dumpCmdPos[i].put(CPPos[i], (CPTicks));
			dumpCmdVel[i].put(CPVel[i], (CPTicks));
			dumpCmdAge[i].put((CPTicks - RPITicks[i]) / 1e9, (CPTicks));
			dumpCmdSmoothAge[i].put((CPTicks - realTicks) / 1e9, (CPTicks));
		}
	}

	std::list<RPI::CrashDumper*> CyclicPositionRobotArm::getCrashDumpers() const
	{
		std::list<RPI::CrashDumper*> ret;
		for(int i=0; i<jointCount; i++) {
			ret.push_back(&dumpRPIPos[i]);
			ret.push_back(&dumpRPIVel[i]);
			ret.push_back(&dumpCmdPos[i]);
			ret.push_back(&dumpCmdVel[i]);
			ret.push_back(&dumpCmdAge[i]);
			ret.push_back(&dumpCmdSmoothAge[i]);
		}
		return ret;
	}


} /* namespace robotarm */
