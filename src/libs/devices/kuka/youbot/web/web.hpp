/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef WEB_HPP_
#define WEB_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

#include <rcc/Module.hpp>
#include <rcc/Device.hpp>
#include <rcc/Server/HTTPServer.hpp>
#include <rcc/Registry.hpp>
#include <rcc/DeviceFactory.hpp>

#include "../device/youBotArm.hpp"
#include "../device/youBotBase.hpp"
#include "../kinematics/yb_Kin.hpp"

namespace kuka_youbot
{
	class RTT_EXPORT ybControllerHandler: public RPI::HTTPHandler
	{
	public:
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data,
				std::string get);
	};

	class RTT_EXPORT ybBaseHandler: public RPI::HTTPHandler
	{
	public:
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data,
				std::string get);
	};

	class RTT_EXPORT ybStatsHandler: public RPI::HTTPHandler
	{
	public:
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data,
				std::string get);
	};

} /* namespace kuka_youbot */
#endif /* WEB_HPP_ */
