/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef INVKINABCZ_HPP_
#define INVKINABCZ_HPP_

#include <kdl/frames.hpp>
#include <rtt/rtt-config.h>

namespace kuka_youbot
{

	class RTT_EXPORT InvKinABCZ
	{
	public:
		InvKinABCZ();
		virtual ~InvKinABCZ();

		void invKin(KDL::Frame pos, double& jnt1, double& jnt2, double& jnt3, double& jnt4, double& jnt5);

	private:

		double hypot(double x, double y);
	};

} /* namespace kuka_youbot */
#endif /* INVKINABCZ_HPP_ */
