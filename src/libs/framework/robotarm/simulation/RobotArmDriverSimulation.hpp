/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ROBOTARMDRIVERSIMULATION_HPP_
#define ROBOTARMDRIVERSIMULATION_HPP_

#include <istream>
#include <list>
#include "../interface/RobotArmInterface.hpp"
#include "rcc/RoundRobinLog.hpp"

namespace robotarm
{
	class RTT_EXPORT RobotArmDriverSimulation: virtual public robotarm::RobotArmInterface
	{
	public:
		RobotArmDriverSimulation(unsigned int jointcount);
		virtual ~RobotArmDriverSimulation();

		virtual int getJointError(int joint);
		virtual void setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time);

		virtual double getMeasuredJointPosition(int joint);
		virtual double getCommandedJointPosition(int joint);

		virtual double getMeasuredJointVelocity(int joint);
		virtual double getCommandedJointVelocity(int joint);

		virtual double getMeasuredJointAcceleration(int joint);
		virtual double getCommandedJointAcceleration(int joint);

		virtual void setToolCOM(KDL::Vector com, int axis);
		virtual void setToolMOI(KDL::Vector moi, int axis);
		virtual void setToolMass(double mass, int axis);
		virtual bool getToolFinished(int axis) const;
		virtual int getToolError(int axis) const;

		virtual bool getPower() const = 0;
		virtual std::list<RPI::CrashDumper*> getCrashDumpers() const;
	private:
		const unsigned int jointcount;

		double *jointpos, *jointvel, *jointacc;
		RTT::os::TimeService::nsecs *lastticks;

		RPI::RoundRobinLog<double>* dumpPos;
		RPI::RoundRobinLog<double>* dumpVel;
	};

} /* namespace robotarm */
#endif /* ROBOTARMDRIVERSIMULATION_HPP_ */
