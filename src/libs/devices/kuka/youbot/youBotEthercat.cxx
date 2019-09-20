/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <string>
#include <rcc/ExtensionLoader.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/version_bin.h>

#include "web/web.hpp"
#include "ethercat/youBotArmEthercat.hpp"
#include "ethercat/youBotBaseEthercat.hpp"
#include "ethercat/youBotGripperEthercat.hpp"

using namespace RPI;

namespace kuka_youbot
{
	extern "C"
	{
		const std::string ext_kuka_youbot_ec = "kuka_youbot_ethercat";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList arm_handlers;
			arm_handlers.push_back(HTTPHandlerInfo("/", new ybControllerHandler()));
			arm_handlers.push_back(HTTPHandlerInfo("/stats", new ybStatsHandler()));
			info.registerDeviceFactory<youBotArmEthercat>(dev_kuka_youbot_arm_ec, ext_kuka_youbot_ec, arm_handlers);

			HTTPHandlerList base_handlers;
			base_handlers.push_back(HTTPHandlerInfo("/", new ybBaseHandler()));
			info.registerDeviceFactory<youBotBaseEthercat>(dev_kuka_youbot_base_ec, ext_kuka_youbot_ec, base_handlers);

			HTTPHandlerList gripper_handlers;
			info.registerDeviceFactory<youBotGripperEthercat>(dev_kuka_youbot_gripper_ec, ext_kuka_youbot_ec, gripper_handlers);

		}
		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_youbot_ec);
		}
	}
}
