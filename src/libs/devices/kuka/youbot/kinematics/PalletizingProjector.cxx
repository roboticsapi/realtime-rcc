/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "PalletizingProjector.hpp"
#include <iostream>

namespace kuka_youbot
{

	PalletizingProjector::PalletizingProjector()
	{
	}

	PalletizingProjector::~PalletizingProjector()
	{
	}

	KDL::Frame PalletizingProjector::project(KDL::Frame flange, KDL::Frame motionCenter)
	{
		KDL::Frame effective = flange * motionCenter;

		double zdir = effective.M.UnitZ().z();
		if(zdir > 0) {
			KDL::Vector x = KDL::Vector(effective.M.UnitX().x(), effective.M.UnitX().y(), 0);
			KDL::Vector z = KDL::Vector(0, 0, 1);
			KDL::Vector y = z * x;
			effective = KDL::Frame(KDL::Rotation(x, y, z), effective.p);
		} else if(zdir < 0) {
			KDL::Vector x = KDL::Vector(effective.M.UnitX().x(), effective.M.UnitX().y(), 0);
			KDL::Vector z = KDL::Vector(0, 0, -1);
			KDL::Vector y = z * x;
			effective = KDL::Frame(KDL::Rotation(x, y, z), effective.p);
		} else {
			effective = KDL::Frame(effective.M * KDL::Rotation::RotX(KDL::PI/2), effective.p);
		}
		return effective * motionCenter.Inverse();
	}

} /* namespace kuka_youbot */
