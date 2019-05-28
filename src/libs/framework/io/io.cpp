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

#include "rpi/IO.hpp"
#include "web/IOWeb.hpp"

namespace IO
{
	using namespace RPI;


	extern "C"
	{
		const std::string ext_io = "io";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<IOInBool>("IO::InBool", ext_io);
			info.registerModuleFactory<IOOutBool>("IO::OutBool", ext_io);
			info.registerModuleFactory<IOOutBoolSensor>("IO::OutBoolSensor", ext_io);
			info.registerModuleFactory<IOInReal>("IO::InReal", ext_io);
			info.registerModuleFactory<IOOutReal>("IO::OutReal", ext_io);
			info.registerModuleFactory<IOOutRealSensor>("IO::OutRealSensor", ext_io);

			info.registerDeviceInterface<IOInterface>("io", ext_io, new IOWeb());

		}
		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_io);
		}
	}
}
