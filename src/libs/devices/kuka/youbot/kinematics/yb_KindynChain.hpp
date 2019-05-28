/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YB_KINDYNCHAIN_HPP_
#define YB_KINDYNCHAIN_HPP_

#include <kdl/chain.hpp>
#include <rtt/rtt-config.h>

namespace kuka_youbot
{
	class RTT_EXPORT yb_kindyn_chain
	{
	public:
		yb_kindyn_chain();
		virtual ~yb_kindyn_chain();

		static KDL::Chain getYBChain();

	private:


	};

}




#endif /* YB_KINDYNCHAIN_HPP_ */
