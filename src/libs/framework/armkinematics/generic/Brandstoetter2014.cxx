/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 
 * Based on paper by Brandstötter, Angerer and Hofbaur 2014
 */

#include "Brandstoetter2014.hpp"

namespace armkinematics
{

	Brandstoetter2014::Brandstoetter2014(double a1, double a2, double b, double c1, double c2, double c3, double c4)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->b = b;
		this->c1 = c1;
		this->c2 = c2;
		this->c3 = c3;
		this->c4 = c4;

	}

	Brandstoetter2014::~Brandstoetter2014()
	{
		// TODO Auto-generated destructor stub
	}

	void Brandstoetter2014::invKin(double t16[8][6], const KDL::Frame& frame) const
	{
		double t13[4][3];
		calc123(t13, frame);

		for (int i = 0; i < 4; ++i)
		{
			double t46[2][3];
			calc456(t46, frame, t13[i]);
			for (int j = 0; j < 2; ++j)
			{
				for (int k = 0; k < 3; ++k)
				{
					t16[2 * i + j][k] = t13[i][k];
					t16[2 * i + j][k + 3] = t46[j][k];
				}
			}
		}
	}

	void Brandstoetter2014::calc123(double t13[4][3], const KDL::Frame& frame) const
	{
		KDL::Vector c0 = frame.p - frame.M * KDL::Vector(0, 0, 1) * c4;

		double nx1 = sqrt(sq(c0.x()) + sq(c0.y()) - sq(b)) - a1;

		double t1_1 = atan2(c0.y(), c0.x()) - atan2(b, nx1 + a1);
		double t1_2 = atan2(c0.y(), c0.x()) + atan2(b, nx1 + a1) - KDL::PI;

		double s1q = sq(nx1) + sq(c0.z() - c1);
		double s2q = sq(nx1 + 2 * a1) + sq(c0.z() - c1);
		double kq = sq(a2) + sq(c3);

		double t2_1 = -acos((s1q + sq(c2) - kq) / (2 * sqrt(s1q) * c2)) + atan2(nx1, c0.z() - c1);
		double t2_2 = +acos((s1q + sq(c2) - kq) / (2 * sqrt(s1q) * c2)) + atan2(nx1, c0.z() - c1);
		double t2_3 = -acos((s2q + sq(c2) - kq) / (2 * sqrt(s2q) * c2)) - atan2(nx1 + 2 * a1, c0.z() - c1);
		double t2_4 = +acos((s2q + sq(c2) - kq) / (2 * sqrt(s2q) * c2)) - atan2(nx1 + 2 * a1, c0.z() - c1);

		double t3_1 = +acos((s1q - sq(c2) - kq) / (2 * c2 * sqrt(kq))) - atan2(a2, c3);
		double t3_2 = -acos((s1q - sq(c2) - kq) / (2 * c2 * sqrt(kq))) - atan2(a2, c3);
		double t3_3 = +acos((s2q - sq(c2) - kq) / (2 * c2 * sqrt(kq))) - atan2(a2, c3);
		double t3_4 = -acos((s2q - sq(c2) - kq) / (2 * c2 * sqrt(kq))) - atan2(a2, c3);

		t13[0][0] = t13[1][0] = t1_1;
		t13[2][0] = t13[3][0] = t1_2;

		t13[0][1] = t2_1;
		t13[1][1] = t2_2;
		t13[2][1] = t2_3;
		t13[3][1] = t2_4;

		t13[0][2] = t3_1;
		t13[1][2] = t3_2;
		t13[2][2] = t3_3;
		t13[3][2] = t3_4;

	}

	void Brandstoetter2014::calc456(double t46[2][3], const KDL::Frame& frame, const double t13[3]) const
	{
		double s1p = sin(t13[0]);
		double s23p = sin(t13[1] + t13[2]);
		double c1p = cos(t13[0]);
		double c23p = cos(t13[1] + t13[2]);

		// Rotation(row, column) can be used to access matrix values
		const auto& e = frame.M;

		double mp = e(0, 2) * s23p * c1p + e(1, 2) * s23p * s1p + e(2, 2) * c23p;

		double t4_1 = atan2(e(1, 2) * c1p - e(0, 2) * s1p,
				e(0, 2) * c23p * c1p + e(1, 2) * c23p * s1p - e(2, 2) * s23p);
		double t4_2 = t4_1 + KDL::PI;

		double t5_1 = atan2(sqrt(1 - sq(mp)), mp);
		double t5_2 = -t5_1;

		double t6_1 = atan2(e(0, 1) * s23p * c1p + e(1, 1) * s23p * s1p + e(2, 1) * c23p,
				-e(0, 0) * s23p * c1p - e(1, 0) * s23p * s1p - e(2, 0) * c23p);
		double t6_2 = t6_1 - KDL::PI;

		t46[0][0] = t4_1;
		t46[1][0] = t4_2;

		t46[0][1] = t5_1;
		t46[1][1] = t5_2;

		t46[0][2] = t6_1;
		t46[1][2] = t6_2;
	}

	double inline Brandstoetter2014::sq(double x) const
	{
		return x * x;
	}

} /* namespace armkinematics */
