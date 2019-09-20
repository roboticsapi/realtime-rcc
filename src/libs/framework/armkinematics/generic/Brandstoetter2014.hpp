/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SRC_LIBS_ARMKINEMATICS_GENERIC_BRANDSTOETTER2014_HPP_
#define SRC_LIBS_ARMKINEMATICS_GENERIC_BRANDSTOETTER2014_HPP_

#include <set>
#include <vector>
#include <kdl/frames.hpp>
#include <rtt/rtt-config.h>

namespace armkinematics
{

	class RTT_EXPORT Brandstoetter2014
	{
	public:
		Brandstoetter2014(double a1, double a2, double b, double c1, double c2, double c3, double c4);
		virtual ~Brandstoetter2014();

		void invKin(double th16[8][6], const KDL::Frame& frame) const;
	private:
		double c1, c2, c3, c4, a1, a2, b;

		void calc123(double th13[4][3], const KDL::Frame& frame) const;
		void calc456(double th46[2][3], const KDL::Frame& frame, const double th13[4]) const;

		double inline sq(double x) const;
	};

} /* namespace armkinematics */

#endif /* SRC_LIBS_ARMKINEMATICS_GENERIC_BRANDSTOETTER2014_HPP_ */
