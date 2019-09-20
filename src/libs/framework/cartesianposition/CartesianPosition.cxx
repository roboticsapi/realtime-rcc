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

#include "rpi/CartesianPositionBlocks.cxx"
#include "web/CartesianPositionHandler.hpp"

namespace cartesianposition
{

	extern "C"
	{

		const std::string ext_cartesianposition = "cartesianposition";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<CartesianPosition>("CartesianPosition::CartesianPosition", ext_cartesianposition);
			info.registerModuleFactory<CartesianMonitor>("CartesianPosition::CartesianMonitor", ext_cartesianposition);

			info.registerDeviceInterface<CartesianPositionInterface>("cartesianposition", ext_cartesianposition, 0, new CartesianPositionHandler());
		}

		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_cartesianposition);
		}

	}
}

