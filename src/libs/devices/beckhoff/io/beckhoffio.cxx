/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/Extension.hpp>
#include <rcc/Module.hpp>
#include <rcc/DeviceFactory.hpp>

#include "device/DummyDevice.hpp"
#include "device/EL1008.hpp"
#include "device/EL2008.hpp"
#include "device/EL2024.hpp"
#include "device/EL4004.hpp"
#include "device/EL3064.hpp"

#include <rcc/version_bin.h>

namespace beckhoffio
{
	using namespace RPI;

	extern "C"
	{
		const std::string ext_beckhoffio = "beckhoffio";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList handlers;
			info.registerDeviceFactory<DummyDevice>(dev_dummy, ext_beckhoffio, handlers);
			info.registerDeviceFactory<EL1008>(dev_el1008, ext_beckhoffio, handlers);
			info.registerDeviceFactory<EL2008>(dev_el2008, ext_beckhoffio, handlers);
			info.registerDeviceFactory<EL2024>(dev_el2024, ext_beckhoffio, handlers);
			info.registerDeviceFactory<EL4004>(dev_el4004, ext_beckhoffio, handlers);
			info.registerDeviceFactory<EL3064>(dev_el3064, ext_beckhoffio, handlers);
		}
		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_beckhoffio);
		}
	}
}
