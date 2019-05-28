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
#include "simulation/youBotArm.hpp"
#include "simulation/youBotBase.hpp"
#include "simulation/youBotGripper.hpp"

using namespace RPI;

namespace kuka_youbot
{
	extern "C"
	{
		const std::string ext_kuka_youbot_sim = "kuka_youbot_simulation";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList handlers;
			handlers.push_back(HTTPHandlerInfo("/", new ybControllerHandler()));
			handlers.push_back(HTTPHandlerInfo("/stats", new ybStatsHandler()));
			info.registerDeviceFactory<youBotArmSimulation>(dev_kuka_youbot_arm_sim, ext_kuka_youbot_sim, handlers);

			HTTPHandlerList base_handlers;
			base_handlers.push_back(HTTPHandlerInfo("/", new ybBaseHandler()));
			info.registerDeviceFactory<youBotBaseSimulation>(dev_kuka_youbot_base_sim, ext_kuka_youbot_sim, base_handlers);

			HTTPHandlerList gripper_handlers;
			info.registerDeviceFactory<youBotGripperSimulation>(dev_kuka_youbot_gripper_sim, ext_kuka_youbot_sim, gripper_handlers);


		}
		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_youbot_sim);
		}
	}
}
