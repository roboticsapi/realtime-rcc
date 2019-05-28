/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "IiwaDevice.hpp"
#include "../kinematics/Lbr_Kin.hpp"

namespace kuka_iiwa
{
	static const KDL::Frame iiwarot = KDL::Frame(KDL::Rotation::RPY(0, 0, KDL::PI));

	IiwaDevice::IiwaDevice(std::string name, parameter_t parameters) :
			LwrDevice(name, parameters)
	{
	}

	KDL::Frame IiwaDevice::Kin(const RPI::Array<double>& joints)
	{
		return iiwarot * LwrDevice::Kin(joints);
	}

	void IiwaDevice::InvKin(const RPI::Array<double>& hintjoints, const KDL::Frame& position,
			RPI::Array<double>& resultJoints)
	{
		KDL::Frame iiwaposition = iiwarot * position;

		LwrDevice::InvKin(hintjoints, iiwaposition, resultJoints);
	}

	void IiwaDevice::alphaKin(double j0, double j1, double j2, double j3, double j4, double j5, double j6,
			KDL::Frame& pos, double& alpha)
	{
		LwrDevice::alphaKin(j0, j1, j2, j3, j4, j5, j6, pos, alpha);

		pos = iiwarot * pos;
	}
	void IiwaDevice::alphaInvKin(KDL::Frame pos, double alpha, double& j0, double& j1, double& j2, double& j3,
			double& j4, double& j5, double& j6)
	{
		KDL::Frame iiwapos = iiwarot * pos;
		LwrDevice::alphaInvKin(iiwapos, alpha, j0, j1, j2, j3, j4, j5, j6);
	}
	void IiwaDevice::alphaVelKin(double j0, double j1, double j2, double j3, double j4, double j5, double j6, double v0,
			double v1, double v2, double v3, double v4, double v5, double v6, KDL::Twist& twist, double& alpha)
	{
		// TODO: Rotate coordinate system
		double l0, l1, l2, l3, l4, l5, l6, l7;
		getLinkLengths(l0, l1, l2, l3, l4, l5, l6, l7);
		Lbr_Kin::lbrVelKin(j0, j1, j2, j3, j4, j5, j6, v0, v1, v2, v3, v4, v5, v6, twist, alpha, l0, l1, l2, l3, l4, l5,
				l6, l7);
	}

}

