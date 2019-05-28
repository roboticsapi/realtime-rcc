/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef POSITIONPRESERVINGPROJECTOR_HPP_
#define POSITIONPRESERVINGPROJECTOR_HPP_

#include <kdl/frames.hpp>

namespace kuka_youbot
{

	class PositionPreservingProjector
	{
	public:
		PositionPreservingProjector();
		virtual ~PositionPreservingProjector();

		KDL::Frame project(KDL::Frame flange, KDL::Frame motionCenter);

	private:
		double angle2D(KDL::Vector x, KDL::Vector y);
	};

} /* namespace kuka_youbot */
#endif /* POSITIONPRESERVINGPROJECTOR_HPP_ */
