/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ROBOTBASEDRIVERSIMULATION_HPP_
#define ROBOTBASEDRIVERSIMULATION_HPP_

#include <rtt/rtt-config.h>
#include <rtt/TaskContext.hpp>
#include <rtt/os/TimeService.hpp>
#include "../interface/RobotBaseInterface.hpp"
#include <libs/framework/cartesianposition/device/CartesianPositionDevice.hpp>

namespace robotbase
{
	#define RBDS_CYCLETIME 0.01

	class RTT_EXPORT RobotBaseDriverSimulation:
		public virtual RobotBaseInterface,
		public virtual cartesianposition::CartesianPositionDevice,
		public RTT::TaskContext
	{
	public:
		RobotBaseDriverSimulation();
		virtual ~RobotBaseDriverSimulation();

		// Interface TaskContext
		virtual void updateHook();

		// Interface CartesianPosition
		virtual int checkPosition(KDL::Frame position);
		virtual KDL::Frame getMeasuredPosition();
		virtual KDL::Twist getMeasuredVelocity();

		// Interface RobotBase
		virtual void setBaseVelocity(KDL::Twist velocity);

		virtual KDL::Frame getMeasuredBasePosition();
		virtual KDL::Twist getMeasuredBaseVelocity();
		virtual KDL::Twist getCommandedBaseVelocity();

		virtual int getWheelCount() const;
		virtual double getWheelPosition(int wheel) const;
		virtual double getWheelVelocity(int wheel) const;
	private:
		KDL::Twist vel;
		RTT::os::TimeService::nsecs lastVel;
	};

} /* namespace robotbase */
#endif /* ROBOTBASEDRIVERSIMULATION_HPP_ */
