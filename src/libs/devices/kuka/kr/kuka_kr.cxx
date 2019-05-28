/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <rcc/Extension.hpp>
#include <rcc/Module.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/version_bin.h>

#include "device/KR_ExtAxis_Device.hpp"

namespace kuka_kr
{
	using namespace RPI;

	extern "C"
	{
		const std::string ext_kuka_kr = "kuka_kr";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList handlers;
			info.registerDeviceFactory<KR_ExtAxis_Device>(KR_ExtAxis_Device::ext_devicename, ext_kuka_kr, handlers);

		}

		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_kr);
		}
	}
}
