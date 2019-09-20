/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "LwrDevice.hpp"
#include "../kinematics/Lbr_Kin.hpp"

namespace kuka_lwr
{
	LwrDevice::LwrDevice(std::string name, RPI::parameter_t parameters) :
			RPI::Device(name, parameters)
	{
		link_lengths = RPI::Module::parseString<double>(getParameter("link_lengths"));
	}

	KDL::Frame LwrDevice::Kin(const RPI::Array<double>& joints)
	{
		KDL::Frame ret;
		double jn[7];
		for (int i = 0; i < 7; ++i)
			jn[i] = joints.get(i);
		double alpha;

		double l0, l1, l2, l3, l4, l5, l6, l7;

		getLinkLengths(l0, l1, l2, l3, l4, l5, l6, l7);

		Lbr_Kin::lbrKin(jn[0], jn[1], jn[2], jn[3], jn[4], jn[5], jn[6], ret, alpha, l0, l1, l2, l3,
				l4, l5, l6, l7);

		return ret;
	}

	void LwrDevice::InvKin(const RPI::Array<double>& hintjoints, const KDL::Frame& position,
			RPI::Array<double>& resultJoints)
	{
		double jn[7];
		for (int i = 0; i < 7; ++i)
			jn[i] = hintjoints.get(i);

		// calculate alpha value for hintjoints
		KDL::Frame tmppos;
		double alpha;
		double l0, l1, l2, l3, l4, l5, l6, l7;
		getLinkLengths(l0, l1, l2, l3, l4, l5, l6, l7);

		Lbr_Kin::lbrKin(jn[0], jn[1], jn[2], jn[3], jn[4], jn[5], jn[6], tmppos, alpha, l0, l1, l2, l3,
				l4, l5, l6, l7);

		Lbr_Kin::lbrInvKin(position, alpha, jn[0], jn[1], jn[2], jn[3], jn[4], jn[5], jn[6], l0, l1, l2,
				l3, l4, l5, l6, l7);
		for (int i = 0; i < 7; ++i)
			resultJoints[i] = jn[i];
	}

	void LwrDevice::alphaKin(double j0, double j1, double j2, double j3, double j4, double j5, double j6, KDL::Frame& pos, double& alpha)
	{
		double l0, l1, l2, l3, l4, l5, l6, l7;
		getLinkLengths(l0, l1, l2, l3, l4, l5, l6, l7);

		Lbr_Kin::lbrKin(j0, j1, j2, j3, j4, j5, j6, pos, alpha, l0, l1, l2, l3,
				l4, l5, l6, l7);
	}
	void LwrDevice::alphaInvKin(KDL::Frame pos, double alpha, double& j0, double& j1, double& j2, double& j3, double& j4, double& j5, double& j6)
	{
		double l0, l1, l2, l3, l4, l5, l6, l7;
		getLinkLengths(l0, l1, l2, l3, l4, l5, l6, l7);

		Lbr_Kin::lbrInvKin(pos, alpha, j0, j1, j2, j3, j4, j5, j6, l0, l1, l2,l3, l4, l5, l6, l7);
	}
	void LwrDevice::alphaVelKin(double j0, double j1, double j2, double j3, double j4, double j5, double j6,
					double v0, double v1, double v2, double v3, double v4, double v5, double v6, KDL::Twist& twist, double& alpha)
	{
		double l0, l1, l2, l3, l4, l5, l6, l7;
		getLinkLengths(l0, l1, l2, l3, l4, l5, l6, l7);
		Lbr_Kin::lbrVelKin(j0, j1, j2, j3, j4, j5, j6, v0, v1, v2, v3, v4, v5, v6, twist, alpha, l0, l1, l2,l3, l4, l5, l6, l7);
	}

	void LwrDevice::getLinkLengths(double& l0, double& l1, double& l2,  double& l3, double& l4, double& l5, double& l6, double& l7)
	{
		l0 = link_lengths[0];
		l1 = link_lengths[1];
		l2 = link_lengths[2];
		l3 = link_lengths[3];
		l4 = link_lengths[4];
		l5 = link_lengths[5];
		l6 = link_lengths[6];
		l7 = link_lengths[7];
	}
} /* namespace FRI */
