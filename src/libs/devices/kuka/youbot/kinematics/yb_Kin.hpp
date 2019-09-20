/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOT_KIN_HPP_
#define YOUBOT_KIN_HPP_

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <rtt/rtt-config.h>

namespace kuka_youbot
{
	class RTT_EXPORT yb_kin
	{
	public:
		yb_kin();
		virtual ~yb_kin();

		void ybInvKin(KDL::Frame pos, double& jnt1, double& jnt2, double& jnt3, double& jnt4, double& jnt5);
		void ybKin(double jnt1, double jnt2, double jnt3, double jnt4, double jnt5, KDL::Frame& pos);
		void ybKinVel(const KDL::JntArrayVel& q_in, KDL::FrameVel& q_out);

	private:
		KDL::Chain chain;

		bool ik(const KDL::Frame& g0, int solution, double& jnt1, double& jnt2, double& jnt3, double& jnt4, double& jnt5);

		const unsigned int jointcount;
	};

}

#endif /* YOUBOT_KIN_HPP_ */
