/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "PositionPreservingProjector.hpp"
#include <iostream>

namespace kuka_youbot
{

	PositionPreservingProjector::PositionPreservingProjector()
	{
	}

	PositionPreservingProjector::~PositionPreservingProjector()
	{
	}

	KDL::Frame PositionPreservingProjector::project(KDL::Frame flange, KDL::Frame motionCenter)
	{
		KDL::Vector f = flange.p;
		KDL::Vector v = flange.M.UnitZ();
		KDL::Frame mcp = flange * motionCenter;
		KDL::Vector m = mcp.p;
		KDL::Vector fm = m - flange.p;

		// flange position vector in XY plane
		f = KDL::Vector(f.x(), f.y(), 0);
		// flange orientation unit Z vector in XY plane
		v = KDL::Vector(v.x(), v.y(), 0);
		// motion center vector in XY plane
		m = KDL::Vector(m.x(), m.y(), 0);
		// flange to motion center vector in XY plane
		fm = KDL::Vector(fm.x(), fm.y(), 0);

		// v points up or down - everything's fine
		if(v.Norm() < 0.001) return flange;

		// f and v are parallel - everything's fine
		if((f * v).Norm() < 0.001) return flange;

		// find out how far to rotate
		double rot = 0;
		if(fm.Norm() < 0.001) {
			// m is zero - just turn v to point towards f
			rot = angle2D(f, v);
			while(rot < -KDL::PI/2) rot += KDL::PI;
			while(rot > KDL::PI/2) rot -= KDL::PI;
		} else {
			// m is not zero - calculate rotation angle
			double rot1, rot2;
			{
				// looking at the triangle O-f-m
				// the angle O-f-m should be equal to the one between v and fm
				double angle = angle2D(v, fm);

				// compute the other angles
				double angle2 = asin(fm.Norm() / m.Norm() * sin(fabs(angle)));
				double angle3 = KDL::PI - fabs(angle) - angle2;

				if(angle < 0) angle3 = -angle3;
				double curAngle = angle2D(-fm, -m);

				rot1 = angle3 - curAngle;
				while(rot < -KDL::PI) rot += 2*KDL::PI;
				while(rot > KDL::PI) rot -= 2*KDL::PI;
			}

			{
				// looking at the triangle O-f-m
				// the angle O-f-m should be equal to the one between v and fm
				double angle = angle2D(-v, fm);

				// compute the other angles
				double angle2 = asin(fm.Norm() / m.Norm() * sin(fabs(angle)));
				double angle3 = KDL::PI - fabs(angle) - angle2;

				if(angle < 0) angle3 = -angle3;
				double curAngle = angle2D(-fm, -m);

				rot2 = angle3 - curAngle;
				while(rot < -KDL::PI) rot += 2*KDL::PI;
				while(rot > KDL::PI) rot -= 2*KDL::PI;
			}

			if(fabs(rot1) < fabs(rot2)) {
				rot = rot1;
			} else {
				rot = rot2;
			}
		}
//		std::cout << "Rotating by " << rot << std::endl;
		KDL::Frame ret = mcp *
				KDL::Frame(mcp.M.Inverse() * KDL::Rotation::RotZ(rot) * mcp.M) *
				motionCenter.Inverse();
//		std::cout << "Before: " << (flange * motionCenter).p.x() << " " <<  (flange * motionCenter).p.y() << "  " << flange.M.UnitZ().x() << " " << flange.M.UnitZ().y() << std::endl;
//		std::cout << "After: " << (ret * motionCenter).p.x() << " " << (ret * motionCenter).p.y() << "  " << ret.M.UnitZ().x() << " " << ret.M.UnitZ().y() << std::endl;
		return ret;
	}

	double PositionPreservingProjector::angle2D(KDL::Vector x, KDL::Vector y) {
		return atan2(x.y(), x.x()) - atan2(y.y(), y.x());
	}

} /* namespace kuka_youbot */
