/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "base_Kin.hpp"

namespace kuka_youbot
{

	base_Kin::base_Kin()
	{

		double lengthBetweenFrontAndRearWheels = 0.47;
		double lengthBetweenFrontWheels = 0.3;
		double wheelRadius = 0.0475;
		double slideRatio = 1;

		FORWARD_RAD_PER_M = 1 / wheelRadius;
		LEFT_RAD_PER_M = 1 / wheelRadius / slideRatio;
		YAW_RAD_PER_RAD =
				((lengthBetweenFrontAndRearWheels + lengthBetweenFrontWheels)
						/ (2.0 * wheelRadius));
	}

	base_Kin::~base_Kin()
	{
	}

	void base_Kin::posKin(double flDelta, double frDelta, double rlDelta, double rrDelta, double& forwardDelta, double& leftDelta, double& yawDelta)
	{
		forwardDelta = (flDelta + frDelta) / 2 / FORWARD_RAD_PER_M;
		leftDelta = -(flDelta - rlDelta) / 2 / LEFT_RAD_PER_M;
		yawDelta = -(flDelta - rrDelta) / 2 / YAW_RAD_PER_RAD;
	}

	void base_Kin::velInvKin(double forwardVel, double leftVel, double yawVel, double& flVel, double& frVel, double& rlVel, double& rrVel)
	{
		double forward = forwardVel * FORWARD_RAD_PER_M,
				left = leftVel * LEFT_RAD_PER_M,
				yaw = yawVel * YAW_RAD_PER_RAD;

		flVel = forward - left - yaw;
		frVel = forward + left + yaw;
		rlVel = forward + left - yaw;
		rrVel = forward - left + yaw;
	}

	void base_Kin::velKin(double flVel, double frVel, double rlVel, double rrVel, double& forwardVel, double& leftVel, double& yawVel)
	{
		forwardVel = (flVel + frVel) / 2 / FORWARD_RAD_PER_M;
		leftVel = -(flVel - rlVel) / 2 / LEFT_RAD_PER_M;
		yawVel = -(flVel - rrVel) / 2 / YAW_RAD_PER_RAD;
	}

} /* namespace kuka_youbot */
