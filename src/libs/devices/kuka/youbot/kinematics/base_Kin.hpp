/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef BASE_KIN_HPP_
#define BASE_KIN_HPP_

#include <rtt/rtt-config.h>

namespace kuka_youbot
{

	class RTT_EXPORT base_Kin
	{
	public:
		base_Kin();
		virtual ~base_Kin();

		void posKin(double flDelta, double frDelta, double rlDelta, double rrDelta, double& forwardDelta, double& leftDelta, double& yawDelta);

		void velInvKin(double forwardVel, double leftVel, double yawVel, double& flVel, double& frVel, double& rlVel, double& rrVel);
		void velKin(double flVel, double frVel, double rlVel, double rrVel, double& forwardVel, double& leftVel, double& yawVel);

	private:
		double FORWARD_RAD_PER_M, LEFT_RAD_PER_M, YAW_RAD_PER_RAD;
	};

} /* namespace kuka_youbot */
#endif /* BASE_KIN_HPP_ */
