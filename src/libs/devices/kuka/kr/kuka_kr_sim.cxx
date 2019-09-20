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
#include "simulation/KR_Arm_Sim.hpp"


namespace kuka_kr
{
	using namespace RPI;

	extern "C"
	{
		const std::string ext_kuka_kr_sim = "kuka_kr_sim";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			//HTTPServer::getInstance()->addHandler("/extensions/staublisim/", new staubliHandler(ext_staublisim));

			HTTPHandlerList handlers;
			//handlers.push_back(HTTPHandlerInfo("/", new staubliControllerHandler()));
			//info.registerDeviceFactory<KR_Simulation>(KR_Simulation::kr_devicename, ext_kuka_kr_sim, handlers);

			info.registerDeviceFactory<KR_Arm_Sim>(KR_Arm_Sim::kr_arm_sim_devicename, ext_kuka_kr_sim, handlers);

		}
		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_kr_sim);
		}
	}
}
