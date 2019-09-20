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


	class Schunk_Kin_Calculation
	{
	public:
		Schunk_Kin_Calculation(dh_parameters const& dh_parameters, const KDL::JntArray& min, const KDL::JntArray& max);
		virtual ~Schunk_Kin_Calculation();

		KDL::Frame Kin(const RPI::Array<double>& joints);
		void InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position, RPI::Array<double>& values);

		//void rsiKinVel(const KDL::JntArrayVel& q_in, KDL::FrameVel& q_out);

	private:
		KDL::Chain chain;
		KDL::JntArray q_min, q_max;
		unsigned int jointcount;
	};

}

#endif /* RSI_KIN_HPP_ */
