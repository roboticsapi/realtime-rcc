/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef LBR_KIN_HPP_
#define LBR_KIN_HPP_

#include <kdl/frames.hpp>
#include <rtt/Logger.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

namespace kuka_lwr
{

	class RTT_EXPORT Lbr_Kin
	{
	public:
		virtual ~Lbr_Kin();

		static void lbrInvKin(KDL::Frame pos, double alpha, double& jnt1, double& jnt2, double& jnt3, double& jnt4,
				double& jnt5, double& jnt6, double& jnt7,
				double l0, double l1, double l2, double l3, double l4,	double l5, double l6, double l7);
		static void lbrKin(double jnt1, double jnt2, double jnt3, double jnt4, double jnt5, double jnt6, double jnt7,
				KDL::Frame& pos, double& alpha,
				double l0, double l1, double l2, double l3, double l4,double l5, double l6, double l7);
		static void lbrVelKin(double jnt1, double jnt2, double jnt3, double jnt4, double jnt5, double jnt6, double jnt7,
				double vel1, double vel2, double vel3, double vel4, double vel5, double vel6, double vel7, KDL::Twist& vel, double& velalpha,
				double l0, double l1, double l2, double l3, double l4,	double l5, double l6, double l7);

	private:
		Lbr_Kin();
	};

}

#endif /* LBR_KIN_HPP_ */
