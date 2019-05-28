/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "Schunk_Kin.hpp"

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <iostream>

using namespace KDL;

namespace schunk_lwa
{

	Schunk_Kin_Calculation::Schunk_Kin_Calculation(dh_parameters const& dh_parameters, const JntArray& min, const JntArray& max)
	{
		jointcount = dh_parameters.dh_d.size();
		if (dh_parameters.dh_a.size() < jointcount)
			jointcount = dh_parameters.dh_a.size();
		if (dh_parameters.dh_al.size() < jointcount)
			jointcount = dh_parameters.dh_al.size();
		if (dh_parameters.dh_t.size() < jointcount)
			jointcount = dh_parameters.dh_t.size();

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

	Schunk_Kin_Calculation::~Schunk_Kin_Calculation()
	{

	}

	KDL::Frame Schunk_Kin_Calculation::Kin(const RPI::Array<double>& joints)
	{
		JntArray axis(6);
		for(long i = 0; i < 6; ++i)
			axis(i) = joints.get(i);

		KDL::Frame pos;

		ChainFkSolverPos_recursive fk(chain);

		fk.JntToCart(axis, pos);

		return pos;
	}

	void Schunk_Kin_Calculation::InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position,
			RPI::Array<double>& values)
	{
		ChainFkSolverPos_recursive fksolver(chain);
		ChainIkSolverVel_pinv iksolvervel(chain);
		ChainIkSolverPos* nr;

		if(q_min.rows() == jointcount && q_max.rows() == jointcount)
			nr = new ChainIkSolverPos_NR_JL(chain, q_min, q_max, fksolver, iksolvervel);
		else
			nr = new ChainIkSolverPos_NR(chain, fksolver, iksolvervel);

		JntArray init(6), out(6);

		for(int i = 0; i < 6; ++i)
			init(i) = hintJoints.get(i);

		nr->CartToJnt(init, position, out);

		// "normalize" output
		for (int i = 0; i < 6; i++)
		{
			double diff = (out(i) > 0) ? -2 * PI : 2 * PI;
			while (fabs(out(i)) > PI)
				out(i) += diff;
		}

		for(int i = 0; i < 6; ++i)
		{
			values[i] = out(i);
		}

		delete nr;
	}

//	void RSI_Kin_Calculation::rsiKinVel(const KDL::JntArrayVel& q_in, KDL::FrameVel& q_out)
//	{
//		ChainFkSolverVel_recursive fksolver(chain);
//
//		fksolver.JntToCart(q_in, q_out);
//	}
}
;
