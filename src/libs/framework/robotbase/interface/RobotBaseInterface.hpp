/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ROBOTBASEDRIVER_HPP_
#define ROBOTBASEDRIVER_HPP_

#include <rtt/Time.hpp>
#include <kdl/frames.hpp>
#include <string>

namespace robotbase {

	class RTT_EXPORT RobotBaseInterface
	{

	public:
		virtual ~RobotBaseInterface() {}

		virtual int getBaseError() = 0;
		virtual int checkBaseVelocity(KDL::Twist velocity) = 0;
		virtual void setBaseVelocity(KDL::Twist velocity) = 0;

		virtual int getWheelCount() const = 0;
		virtual double getWheelPosition(int wheel) const = 0;
		virtual double getWheelVelocity(int wheel) const = 0;

		virtual KDL::Frame getMeasuredBasePosition() = 0;
		virtual KDL::Twist getMeasuredBaseVelocity() = 0;
		virtual KDL::Twist getCommandedBaseVelocity() = 0;

		virtual std::string getRobotBaseResourceName() = 0;

	};
}


#endif /* ROBOTBASEDRIVER_HPP_ */
