/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef RSI_KIN_HPP_
#define RSI_KIN_HPP_

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarrayvel.hpp>

#include <templates/TArray.hpp>

namespace schunk_lwa
{

	struct dh_parameters
	{
		std::vector<double> dh_d;
		std::vector<double> dh_t;
		std::vector<double> dh_a;
		std::vector<double> dh_al;
	};


	class RTT_EXPORT Schunk_KinGeo_Calculation
	{
	public:
		Schunk_KinGeo_Calculation(dh_parameters const& dh_parameters, const KDL::JntArray& min, const KDL::JntArray& max);
		virtual ~Schunk_KinGeo_Calculation();

		KDL::Frame Kin(const RPI::Array<double>& joints);
		void InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position, RPI::Array<double>& values);

		std::vector<KDL::JntArray> InvKin8(const KDL::Frame& position);

	private:
		KDL::Chain chain;
		KDL::JntArray q_min, q_max;
		unsigned int jointcount;
		dh_parameters dh_param;

		std::vector<KDL::JntArray> Calculate23(double x, double y, double a1, double a2);
		std::vector<KDL::JntArray> Calculate456(KDL::Rotation r3e);
	};

}

#endif /* RSI_KIN_HPP_ */
