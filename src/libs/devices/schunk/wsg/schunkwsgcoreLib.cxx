/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <string>
#include "interface/SchunkWsgDevice.hpp"
#include <rcc/ExtensionLoader.hpp>
#include <rcc/DeviceFactory.hpp>
#include "rpi/SchunkWsgBlocks.hpp"
#include <rcc/version_bin.h>

using namespace RPI;

namespace schunkwsg
{

	extern "C"
	{
		const std::string ext_schunkwsgcore = "schunk_wsg";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<schunkwsg::Homing>("schunkwsg::Homing", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::Prepositioning>("schunkwsg::Prepositioning", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::Grasping>("schunkwsg::Grasping", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::Releasing>("schunkwsg::Releasing", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::SetAcceleration>("schunkwsg::SetAcceleration", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::SetForceLimit>("schunkwsg::SetForceLimit", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::SetSoftLimits>("schunkwsg::SetSoftLimits", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::ClearSoftLimits>("schunkwsg::ClearSoftLimits", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::StopDevice>("schunkwsg::StopDevice", ext_schunkwsgcore);
			info.registerModuleFactory<schunkwsg::Monitor>("schunkwsg::Monitor", ext_schunkwsgcore);
		}
		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_schunkwsgcore);
		}

	}
}
