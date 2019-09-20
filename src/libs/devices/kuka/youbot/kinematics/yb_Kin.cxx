/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "yb_Kin.hpp"
#include "yb_KindynChain.hpp"

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

//#include "yb_KindynChain.hpp"
#include <iostream>

//using namespace KDL;
//using namespace std;

namespace kuka_youbot
{

	yb_kin::yb_kin():jointcount(5)
	{
		double dh_d[5] =  {-0.147, 0,       0,      0,       -0.171};
		double dh_t[5] =  {0,      -1.5708, 0,      1.5708,  0};
		double dh_a[5] =  {0.033,  0.155,   0.135,  0,       0};
		double dh_al[5] = {1.5708, 0,       0,      -1.5708, 3.142};

		// KS0-Z points into the floor, KS-1 corrects this statically
		chain.addSegment(KDL::Segment(KDL::Joint(), KDL::Frame().DH(0, KDL::PI, 0, 0)));

		for (unsigned int i = 0; i < jointcount; i++)
		{
			KDL::Segment seg(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH(dh_a[i], dh_al[i], dh_d[i], dh_t[i]));
			chain.addSegment(seg);
		}
	}

	yb_kin::~yb_kin()
	{

	}

	void yb_kin::ybKin(double jnt1, double jnt2, double jnt3, double jnt4, double jnt5,
			KDL::Frame& pos)
	{
		KDL::JntArray axis(5);
		axis(0) = jnt1;
		axis(1) = jnt2;
		axis(2) = jnt3;
		axis(3) = jnt4;
		axis(4) = jnt5;

		KDL::ChainFkSolverPos_recursive fk(chain);

		fk.JntToCart(axis, pos);
	}


	void yb_kin::ybInvKin(KDL::Frame pos, double& jnt1, double& jnt2, double& jnt3, double& jnt4, double& jnt5)
	{
		double b1, b2, b3, b4, b5, bestDiff = 999;
		b1 = b2 = b3 = b4 = b5 = std::numeric_limits<double>::quiet_NaN();
		for(int solution=0; solution < 4; solution++) {
			double j1, j2, j3, j4, j5;
			if(ik(pos, solution, j1, j2, j3, j4, j5)) {
				double diff = fabs(j1 - jnt1) + fabs(j2 - jnt2) + fabs(j3 - jnt3) + fabs(j4 - jnt4) + fabs(j5 - jnt5);
				if(diff < bestDiff) {
					bestDiff = diff;
					b1 = j1; b2 = j2; b3 = j3; b4 = j4; b5 = j5;
				}
			}
		}

		jnt1 = b1; jnt2 = b2; jnt3 = b3; jnt4 = b4; jnt5 = b5;
	}


	void yb_kin::ybKinVel(const KDL::JntArrayVel& q_in, KDL::FrameVel& q_out)
	{
		KDL::ChainFkSolverVel_recursive fksolver(chain);

		fksolver.JntToCart(q_in, q_out);
	}

	bool yb_kin::ik(const KDL::Frame& g0, int solution, double& jnt1, double& jnt2, double& jnt3, double& jnt4, double& jnt5)
	{
		double EPS = 0.00001;

		bool offset_joint_1 = (solution & 1) != 0;
		bool offset_joint_3 = (solution & 2) != 0;

		// arm lenghts
		double l0x = 0.0;
		double l0z = 0.147;

		double l1x = 0.033;
		double l1z = 0;

		double l2 = 0.155;
		double l3 = 0.135;

		// Distance from arm_link_3 to arm_link_5 (can also be replaced by e.g.
		// distance from arm_link_3 to tool center point)
		double d = 0.171;

		double j1;
		double j2;
		double j3;
		double j4;
		double j5;

		// Transform from frame 0 to frame 1
		KDL::Frame frame0_to_frame1(KDL::Rotation::Identity(), KDL::Vector(-l0x, 0, -l0z));
		KDL::Frame g1 = frame0_to_frame1 * g0;

		// First joint
		j1 = atan2(g1.p.y(), g1.p.x());
		if (offset_joint_1)
		{
			if (j1 < 0.0)
				j1 += KDL::PI;
			else
				j1 -= KDL::PI;
		}

		// Transform from frame 1 to frame 2
		KDL::Frame frame1_to_frame2(KDL::Rotation::RPY(0, 0, -j1), KDL::Vector(-l1x, 0, -l1z));
		KDL::Frame g2 = frame1_to_frame2 * g1;

		// Check if Z vector of hand is in XZ-plane
		if(fabs(KDL::dot(g2.M.UnitZ(), KDL::Vector(0, 1, 0))) > 0.01)
		{
			return false;
		}

		// Set all values in the frame that are close to zero to exactly zero
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (fabs(g2.M(i, j)) < EPS)
					g2.M(i, j) = 0;
			}
		}

		// Fifth joint, determines the roll of the gripper (= wrist angle)
		double s1 = sin(j1);
		double c1 = cos(j1);
		double r11 = g1.M(0, 0);
		double r12 = g1.M(0, 1);
		double r21 = g1.M(1, 0);
		double r22 = g1.M(1, 1);
		j5 = atan2(r21 * c1 - r11 * s1, r22 * c1 - r12 * s1);

		// The sum of joint angles two to four determines the overall "pitch" of the
		// end effector
		double r13 = g2.M(0, 2);
		double r33 = g2.M(2, 2);
		double j234 = atan2(r13 , r33);

		KDL::Vector p2 = g2.p;

		p2.x(p2.x() - d * sin(j234));
		p2.z(p2.z() - d * cos(j234));

		// Check if the goal position can be reached at all
		if ((l2 + l3) + 1e-3 < sqrt((p2.x() * p2.x()) + (p2.z() * p2.z())))
		{
			return false;
		}

		// Third joint
		double j3_cos = ((p2.x() * p2.x()) + (p2.z() * p2.z()) - (l2 * l2) - (l3 * l3)) / (2 * l2 * l3);
		if (j3_cos > 1.0 - EPS)
			j3 = 0.0;
		else if (j3_cos < -1.0 + EPS)
			j3 = KDL::PI;
		else
			j3 = acos(j3_cos);

		if (offset_joint_3)
			j3 = -j3;

		// Second joint
		double t1 = atan2(-p2.x(), p2.z());
		double t2 = atan2(l3 * sin(j3), l2 + l3 * cos(j3));
		if (j3 >= 0) t2 = -t2;
		if(offset_joint_3) t2 = -t2;
		j2 = -t1 + t2;

		// Fourth joint, determines the pitch of the gripper
		j4 = j234 - j2 - j3;

		// normalize j4
		while(j4 > KDL::PI)
			j4 -= 2*KDL::PI;

		while(j4 < -KDL::PI)
			j4 += 2*KDL::PI;

		jnt1 = -j1;
		jnt2 = j2;
		jnt3 = j3;
		jnt4 = j4;
		jnt5 = -j5;
		return true;
	}



}
