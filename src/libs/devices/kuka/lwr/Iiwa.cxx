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

#include "device/IiwaDevice.hpp"
#include "rpi/controller.hpp"

namespace kuka_iiwa
{

	using namespace RPI;

	extern "C"
	{
		const std::string ext_kuka_iiwa = "kuka_iiwa";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<ControlStrategy>("KUKA::LWR::ControlStrategy", ext_kuka_iiwa);
			info.registerModuleFactory<ToolParameters>("KUKA:LWR::ToolParameters", ext_kuka_iiwa);
		}

		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_iiwa);
		}
	}

}

