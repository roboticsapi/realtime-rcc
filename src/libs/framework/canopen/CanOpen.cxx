/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <string>
#include "driver/CanOpenDevice.hpp"
#include "driver/CanOpenMaster.hpp"
#include <rcc/ExtensionLoader.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/version_bin.h>

using namespace RPI;

namespace canopen
{

	extern "C"
	{
		const std::string ext_canopen = "canopen";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList handlers;
			info.registerDeviceFactory<CanOpenDevice>(dev_canopen, ext_canopen, handlers);
			info.registerDeviceFactory<CanOpenMaster>(dev_canopenmaster, ext_canopen, handlers);

			info.registerDeviceInterface<CanOpenInterface>("canopen", ext_canopen, 0, HTTPHandlerList());
			info.registerDeviceInterface<CanOpenMasterInterface>("canopenmaster", ext_canopen, 0, HTTPHandlerList());
		}
		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_canopen);
		}

	}
}
