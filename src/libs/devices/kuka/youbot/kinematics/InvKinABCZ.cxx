/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "InvKinABCZ.hpp"
#include <limits>

namespace kuka_youbot
{

	InvKinABCZ::InvKinABCZ()
	{
		// TODO Auto-generated constructor stub

	}

	InvKinABCZ::~InvKinABCZ()
	{
		// TODO Auto-generated destructor stub
	}

	void InvKinABCZ::invKin(KDL::Frame pos, double& jnt1, double& jnt2, double& jnt3, double& jnt4, double& jnt5)
	{
		double l0 = 0.147; // z direction
		double l1 = 0.033; // x direction
		double l2 = 0.155; // z direction
		double l3 = 0.135; // z direction
		double l4 = 0.171; // z direction

		double theta1, theta234, theta5;
		pos.M.GetEulerZYZ(theta1, theta234, theta5);
		theta1 = -theta1; theta5 = -theta5;
		if(fabs(theta234) < KDL::epsilon) {
			theta5 = theta1;
			theta1 = -atan2(pos.p.y(), pos.p.x());
			theta5 -= theta1;
		}
		if(fabs(theta234 - KDL::PI) < KDL::epsilon) {
			theta5 = -theta1;
			theta1 = -atan2(pos.p.y(), pos.p.x());
			theta5 += theta1;
		}
		KDL::Vector hwp = pos * KDL::Vector(0, 0, -l4);
		double z = hwp.z() - l0, d = hypot(hwp.x(), hwp.y()) - l1;
		double l23 = l2 + l3;
		if(hypot(z, d) > l23 - 0.001) {
			if(z > l23) {
				jnt1 = jnt2 = jnt3 = jnt4 = jnt5 = std::numeric_limits<double>::quiet_NaN();
				return;
			}
			d = sqrt(l23*l23 - z*z) - 0.001;
		}
		double zd = hypot(z, d);

		double angle = atan2(d, z);
		double gamma = acos((l2*l2 + l3*l3 - zd*zd) / 2/l2/l3);
		double theta2 = angle - acos((l2*l2 + zd*zd - l3*l3) / 2/l2/zd);
		double theta3 = KDL::PI - gamma;
		double theta4 = theta234 - theta2 - theta3;
		jnt1 = theta1;
		jnt2 = theta2;
		jnt3 = theta3;
		jnt4 = theta4;
		jnt5 = theta5;
	}

	double InvKinABCZ::hypot(double x, double y) {
		return sqrt(x*x+y*y);
	}

} /* namespace kuka_youbot */
