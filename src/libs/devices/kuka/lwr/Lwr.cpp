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

#include "rpi/Lbr.hpp"
#include "web/web.hpp"

namespace kuka_lwr
{

	using namespace RPI;

	extern "C"
	{
		const std::string ext_kuka_lwr = "kuka_lwr";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<ControlStrategy>("KUKA::LWR::ControlStrategy", ext_kuka_lwr);
			info.registerModuleFactory<ToolParameters>("KUKA::LWR::ToolParameters", ext_kuka_lwr);
			info.registerModuleFactory<JointImpParameters>("KUKA::LWR::JointImpParameters", ext_kuka_lwr);
			info.registerModuleFactory<CartImpParameters>("KUKA::LWR::CartImpParameters", ext_kuka_lwr);
			info.registerModuleFactory<InvKin>("KUKA::LWR::InvKin", ext_kuka_lwr);
			info.registerModuleFactory<Kin>("KUKA::LWR::Kin", ext_kuka_lwr);
			info.registerModuleFactory<VelKin>("KUKA::LWR::VelKin", ext_kuka_lwr);
			info.registerModuleFactory<FTMonitor>("KUKA::LWR::FTMonitor", ext_kuka_lwr);
			info.registerModuleFactory<FMonitor>("KUKA::LWR::FMonitor", ext_kuka_lwr);
			info.registerModuleFactory<TMonitor>("KUKA::LWR::TMonitor", ext_kuka_lwr);
		}

		void RTT_EXPORT unload(ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_kuka_lwr);
		}
	}

}

