/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "KR_Kin.hpp"

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <iostream>

#include <math.h>

using namespace KDL;

namespace kuka_kr
{
	using namespace std;
	using namespace KDL;

	KR_Kin_Calculation::KR_Kin_Calculation(dh_parameters const& dh_parameters, const JntArray& min, const JntArray& max)
	{
		dh_param = dh_parameters;
		jointcount = dh_parameters.dh_d.size();
		if (dh_parameters.dh_a.size() < jointcount)
			jointcount = dh_parameters.dh_a.size();
		if (dh_parameters.dh_al.size() < jointcount)
			jointcount = dh_parameters.dh_al.size();
		if (dh_parameters.dh_t.size() < jointcount)
			jointcount = dh_parameters.dh_t.size();

		// KS0 has Z-Axis into the floor
		chain.addSegment(Segment(Joint(), Frame().DH(0, PI, 0, 0)));

		for (unsigned int i = 0; i < jointcount; i++)
		{
			Segment seg(Joint(Joint::RotZ),
					Frame().DH(dh_parameters.dh_a[i], dh_parameters.dh_al[i], dh_parameters.dh_d[i],
							dh_parameters.dh_t[i]));
			chain.addSegment(seg);
		}

		q_min = min;
		q_max = max;
	}

	KR_Kin_Calculation::~KR_Kin_Calculation()
	{

	}

	KDL::Frame KR_Kin_Calculation::Kin(const RPI::Array<double>& joints)
	{
		JntArray axis(6);
		for (long i = 0; i < 6; ++i)
			axis(i) = joints.get(i);

		KDL::Frame pos;

		ChainFkSolverPos_recursive fk(chain);

		fk.JntToCart(axis, pos);

		return pos;
	}

	void KR_Kin_Calculation::InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position,
			RPI::Array<double>& values)
	{
		std::vector<KDL::JntArray> res8 = InvKin8(position);

		//		// Todo: Remove, only for debugging
		//		for (int i = 0; i < res8.size(); i++)
		//		{
		//			RPI::Array<double> values;
		//			values.resize(6);
		//
		//			std::cout << "Solution " << (i + 1) << " ";
		//			for (int j = 0; j < 6; j++)
		//			{
		//				std::cout << "j" << j << ": " << res8[i](j) << " ";
		//				values[j] = res8[i](j);
		//			}
		//
		//			std::cout << std::endl;
		//
		//			Frame fk = Kin(values);
		//
		//			std::cout << "Solution " << (i + 1) << " ";
		//			std::cout << "x: " << fk.p.x() << " y: " << fk.p.y() << " z: " << fk.p.z() << std::endl;
		//		}

		// Normalize solutions to [-PI + hint joint, PI + hint joint]
		for (int i = 0; i < res8.size(); i++)
		{
			for (int j = 0; j < 6; j++)
			{
				// Solution found differs more than 180 degrees from hint joints
				double diff;
				while (fabs(diff = res8[i](j) - hintJoints.get(j)) > PI)
				{
					res8[i](j) -= sgn(diff) * 2 * PI;
				}
			}
		}

		// Select valid configurations
		std::vector<KDL::JntArray> valid_config;

		for (int i = 0; i < res8.size(); i++)
		{
			bool valid = true;
			for (int j = 0; j < 6; j++)
			{
				// solution exceeds maximum joint value
				while (res8[i](j) > q_max(j))
				{
					res8[i](j) -= 2 * PI;
				}
				// solution below minimum joint value
				while (res8[i](j) < q_min(j))
				{
					res8[i](j) += 2 * PI;
				}

				// check whether (adjusted) joint value is still inside joint limits
				valid &= (res8[i](j) <= q_max(j));
			}
			if (valid)
			{
				valid_config.push_back(res8[i]);
				//std::cout << "Solution " << (i + 1) << " valid" << std::endl;
			}
		}

		if (valid_config.size() == 0)
		{
			for (int j = 0; j < 6; j++)
				values[j] = numeric_limits<double>::quiet_NaN();

			return;
		}

		// Select best configuration

		int selected = 0;
		double min_sq = numeric_limits<double>::infinity();

		for (int i = 0; i < valid_config.size(); i++)
		{
			double sq = 0;
			for (int j = 0; j < 6; j++)
			{
				auto diff = valid_config[i](j) - hintJoints.get(j);
				sq += diff * diff;
			}

			if (sq < min_sq)
			{
				selected = i;
				min_sq = sq;
			}
		}

		for (int j = 0; j < 6; j++)
		{
			values[j] = valid_config[selected](j);
		}
	}

	std::vector<KDL::JntArray> KR_Kin_Calculation::InvKin8(const Frame& position)
	{
		std::vector<KDL::JntArray> result;

		Frame ourpos = Frame(Rotation().RotX(PI)) * position;

		Frame t5e = Frame().DH(dh_param.dh_a[5], dh_param.dh_al[5], dh_param.dh_d[5], dh_param.dh_t[5]);

		Frame tr5 = ourpos * t5e.Inverse();

		// Two solutions for axis 1
		for (int iq1 = 0; iq1 < 2; ++iq1)
		{
			// Angle 1
			double q1 = ::atan2(tr5.p.y(), tr5.p.x());

			// Second solution?
			if (iq1 == 1)
			{
				q1 += (q1 > 0) ? -PI : PI;
			}

			Frame tr1 = Frame().DH(dh_param.dh_a[0], dh_param.dh_al[0], dh_param.dh_d[0], dh_param.dh_t[0] + q1);
			Frame t15 = tr1.Inverse() * tr5;

			// Calculate Angles 2 and 3 (two solutions)
			std::vector<KDL::JntArray> q23 = Calculate23(t15.p.x(), t15.p.y(), dh_param.dh_a[1], fabs(dh_param.dh_d[3]),
					dh_param.dh_a[2]);

			// Solution found
			if (q23.size() == 2)
			{

				// Two solutions for q23
				for (int iq2 = 0; iq2 < 2; ++iq2)
				{
					Frame t12 = Frame().DH(dh_param.dh_a[1], dh_param.dh_al[1], dh_param.dh_d[1],
							dh_param.dh_t[1] + q23[iq2](0));
					Frame t23 = Frame().DH(dh_param.dh_a[2], dh_param.dh_al[2], dh_param.dh_d[2],
							dh_param.dh_t[2] + q23[iq2](1));

					Frame tr3 = tr1 * t12 * t23;

					Frame t3e = tr3.Inverse() * ourpos;

					std::vector<KDL::JntArray> q456 = Calculate456(t3e.M);

					// Two solutions for q456
					for (int iq4 = 0; iq4 < 2; ++iq4)
					{
						JntArray sol(6);
						sol(0) = q1;
						sol(1) = q23[iq2](0);
						sol(2) = q23[iq2](1);
						sol(3) = q456[iq4](0);
						sol(4) = q456[iq4](1);
						sol(5) = q456[iq4](2);

						result.push_back(sol);
					}
				}
			} else
			{
				// If no solution for first axis 1 solution found, no solutions will ever be found
				if (iq1 == 0)
					return std::vector<KDL::JntArray>();
			}
		}
		return result;
	}

	std::vector<KDL::JntArray> KR_Kin_Calculation::Calculate23(double x, double y, double a1, double a2, double offset)
	{
		double d = ::sqrt(x * x + y * y);
		double a = ::sqrt(a2 * a2 + offset * offset);
		double gamma = ::atan2(offset, a2);

		double v2 = ::acos(((a1 * a1) + (a * a) - (d * d)) / (2 * a1 * a));

		if (v2 != v2)
			return std::vector<KDL::JntArray>();

		JntArray qa(2), qb(2);

		qa(1) = -PI + v2 - gamma;
		qb(1) = PI - v2 - gamma;

		double delta = ::atan2(-y, x);

		double beta = ::acos(((a1 * a1) + (d * d) - (a * a)) / (2 * a1 * d));

		qa(0) = -delta + beta;
		qb(0) = -delta - beta;

		std::vector<KDL::JntArray> result;
		result.push_back(qa);
		result.push_back(qb);

		return result;
	}

	std::vector<KDL::JntArray> KR_Kin_Calculation::Calculate456(KDL::Rotation r3e)
	{
		// adjust the end effector orientation to the orientation of frame 5
		r3e = r3e * Rotation().RPY(PI, 0, 0);

		double sqrtv = ::sqrt(r3e.data[2] * r3e.data[2] + r3e.data[5] * r3e.data[5]);

		JntArray qa(3), qb(3);

		qa(0) = ::atan2(+r3e.data[5], +r3e.data[2]);
		qb(0) = ::atan2(-r3e.data[5], -r3e.data[2]);

		qb(1) = ::atan2(+sqrtv, r3e.data[8]);
		qa(1) = ::atan2(-sqrtv, r3e.data[8]);

		qa(2) = ::atan2(+r3e.data[7], -r3e.data[6]);
		qb(2) = ::atan2(-r3e.data[7], +r3e.data[6]);

		std::vector<KDL::JntArray> result;
		result.push_back(qa);
		result.push_back(qb);

		return result;
	}
}

