/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/Module.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/Extension.hpp>
#include <rcc/ExtensionLoader.hpp>
#include <rcc/version_bin.h>

#include "rpi/RobotArmBlocks.cpp"
#include "web/RobotArmHandler.hpp"

namespace robotarm
{

	extern "C"
	{
		const std::string ext_robotarm = "robotarm";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<JointPosition>("RobotArm::JointPosition", ext_robotarm);
			info.registerModuleFactory<JointMonitor>("RobotArm::JointMonitor", ext_robotarm);
			info.registerModuleFactory<ToolParameters>("RobotArm::ToolParameters", ext_robotarm);

			info.registerDeviceInterface<RobotArmInterface>("robotarm", ext_robotarm, new RobotArmHandler());
		}

		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_robotarm);
		}

	}
}
