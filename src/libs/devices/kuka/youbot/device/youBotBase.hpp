/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTBASE_HPP_
#define YOUBOTBASE_HPP_


#include <rcc/Device.hpp>
#include <rcc/DeviceFactory.hpp>
#include <libs/framework/robotbase/interface/RobotBaseInterface.hpp>
#include <libs/framework/cartesianposition/interface/CartesianPositionInterface.hpp>

namespace kuka_youbot {

	class RTT_EXPORT youBotBase : public virtual robotbase::RobotBaseInterface,
		public virtual cartesianposition::CartesianPositionInterface
	{
	public:
		virtual ~youBotBase() {}

	};

}


#endif /* YOUBOTBASE_HPP_ */
