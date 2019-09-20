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

#include "interface/EthernetServerUDPInterface.hpp"
#include "standard/EthernetServerUDPstandard.hpp"

namespace ethernet
{
	using namespace RPI;

	extern "C"
	{
		const std::string ext_ethernet_standard = "ethernet_standard";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList handlers;
			info.registerDeviceFactory<EthernetServerUDPstandard>(EthernetServerUDPstandard::devicename, ext_ethernet_standard, handlers);

		}
		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_ethernet_standard);
		}
	}
}
