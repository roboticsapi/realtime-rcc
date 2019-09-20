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

#include "rpi/youBotBlocks.hpp"
#include "web/web.hpp"

using namespace RPI;

namespace kuka_youbot
{
	extern "C"
	{
		const std::string ext_kuka_youbot = "kuka_youbot";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<Gripper>("KUKA::youBot::Gripper", ext_kuka_youbot);
			info.registerModuleFactory<GripperMonitor>("KUKA::youBot::GripperMonitor", ext_kuka_youbot);
			info.registerModuleFactory<SlaveError>("KUKA::youBot::SlaveError", ext_kuka_youbot);
			info.registerModuleFactory<PositionPreservingProject>("KUKA::youBot::PositionPreservingProject", ext_kuka_youbot);
			info.registerModuleFactory<PalletizingProject>("KUKA::youBot::PalletizingProject", ext_kuka_youbot);
			info.registerModuleFactory<ArmControlStrategy>("KUKA::youBot::ArmControlStrategy", ext_kuka_youbot);
			info.registerModuleFactory<JointImpParameters>("KUKA::youBot::JointImpParameters", ext_kuka_youbot);
		}

		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_youbot);
		}
	}
}
