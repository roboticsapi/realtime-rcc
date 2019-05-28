/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CARTESIANPOSITIONINTERFACE_HPP_
#define CARTESIANPOSITIONINTERFACE_HPP_

#include <rtt/Time.hpp>
#include <kdl/frames.hpp>
#include <rtt/os/TimeService.hpp>
#include <string>

namespace cartesianposition {

	class RTT_EXPORT CartesianPositionInterface
	{

	public:
		virtual ~CartesianPositionInterface() {}

		virtual std::string getCartesianPositionResourceName() = 0;
		virtual int getCartesianPositionDeviceError() = 0;
		virtual int checkPosition(KDL::Frame position) = 0;
		virtual void setPosition(KDL::Frame position, RTT::os::TimeService::nsecs time) = 0;

		virtual KDL::Frame getMeasuredPosition() = 0;
		virtual KDL::Twist getMeasuredVelocity() = 0;
		virtual KDL::Frame getCommandedPosition() = 0;
		virtual KDL::Twist getCommandedVelocity() = 0;
	};
}


#endif /* CARTESIANPOSITIONINTERFACE_HPP_ */
