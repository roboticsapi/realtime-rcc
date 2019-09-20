/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ROBOTARMDRIVER_HPP_
#define ROBOTARMDRIVER_HPP_

#include <rtt/os/TimeService.hpp>
#include <kdl/frames.hpp>

namespace robotarm {

	enum JointPositionError
	{
		JP_OK = 0,
		JP_OUTOFRANGE,
		JP_INVALID
	};

	class RTT_EXPORT RobotArmInterface
	{

	public:
		virtual ~RobotArmInterface() {};
		virtual int getJointCount() const = 0;

		virtual int getJointError(int joint) = 0;
		virtual void setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time) = 0;
		virtual JointPositionError checkJointPosition(int joint, double position) = 0;

		virtual double getMeasuredJointPosition(int joint) = 0;
		virtual double getCommandedJointPosition(int joint) = 0;

		virtual double getMeasuredJointVelocity(int joint) = 0;
		virtual double getCommandedJointVelocity(int joint) = 0;

		virtual double getMeasuredJointAcceleration(int joint) = 0;
		virtual double getCommandedJointAcceleration(int joint) = 0;

		virtual void setToolCOM(KDL::Vector com, int axis) = 0;
		virtual void setToolMOI(KDL::Vector moi, int axis) = 0;
		virtual void setToolMass(double mass, int axis) = 0;
		virtual bool getToolFinished(int axis) const = 0;
		virtual int getToolError(int axis) const = 0;
	};
}


#endif /* ROBOTARMDRIVER_HPP_ */
