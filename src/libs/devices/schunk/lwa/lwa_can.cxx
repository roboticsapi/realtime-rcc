/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <string>
#include "driver/SchunkLwaDriver.hpp"
#include <rcc/ExtensionLoader.hpp>
#include <rcc/DeviceFactory.hpp>
#include "web/web.hpp"
#include <rcc/version_bin.h>

using namespace RPI;

namespace schunk_lwa
{

	extern "C"
	{
		const std::string ext_schunk_lwa_can = "schunk_lwa_can";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList handlers;
			handlers.push_back(HTTPHandlerInfo("/", new SchunkLWAHandler()));
			info.registerDeviceFactory<SchunkLwaDriver>(dev_schunk_lwa_can, ext_schunk_lwa_can, handlers);
		}
		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_schunk_lwa_can);
		}

	}
}
