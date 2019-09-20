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

#include "web/web.hpp"
#include "simulation/IiwaSimulation.hpp"

namespace kuka_iiwa {

	using namespace RPI;

	extern "C"
	{
		const std::string ext_kuka_iiwa_sim = "kuka_iiwa_sim";//"kuka_lwr_sim";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			HTTPHandlerList handlers;
			handlers.push_back(HTTPHandlerInfo("/", new LBRFRIControllerHandler()));
			handlers.push_back(HTTPHandlerInfo("/kinematics", new LBRFRIKinematicsHandler()));
			info.registerDeviceFactory<IiwaSimulation>(dev_iiwa_joint_sim, ext_kuka_iiwa_sim, handlers);

		}
		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_iiwa_sim);
		}
	}


}
