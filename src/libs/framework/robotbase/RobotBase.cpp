/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <string>
#include <rcc/Module.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/Extension.hpp>
#include <rcc/ExtensionLoader.hpp>
#include <rcc/version_bin.h>

#include "rpi/RobotBaseBlocks.cpp"
#include "web/RobotBaseHandler.hpp"
#include "device/RobotBaseSim.hpp"

namespace robotbase
{

	extern "C"
	{

		const std::string ext_robotbase = "robotbase";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<BaseVelocity>("RobotBase::BaseVelocity", ext_robotbase);
			info.registerModuleFactory<BaseMonitor>("RobotBase::BaseMonitor", ext_robotbase);
			info.registerModuleFactory<WheelMonitor>("RobotBase::WheelMonitor", ext_robotbase);
			info.registerModuleFactory<Controller>("RobotBase::OmnidirectionalBaseController", ext_robotbase);

			info.registerDeviceInterface<RobotBaseInterface>("robotbase", ext_robotbase, new RobotBaseHandler());

			RPI::HTTPHandlerList handlers;
			info.registerDeviceFactory<robotbase::RobotBaseSim>(robotbase::dev_robotbase_sim, ext_robotbase, handlers);

		}

		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_robotbase);
		}

	}
}

