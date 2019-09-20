/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/Module.hpp>
#include <rcc/Extension.hpp>
#include <rcc/DeviceInterface.hpp>

#include "rpi/ArmKinematicBlocks.hpp"
#include "web/ArmKinematicsHandler.hpp"

#include <rcc/version_bin.h>

namespace armkinematics
{

	extern "C"
	{
		const std::string ext_armkinematics = "armkinematics";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			info.registerModuleFactory<Kin>("ArmKinematics::Kin", ext_armkinematics);
			info.registerModuleFactory<InvKin>("ArmKinematics::InvKin", ext_armkinematics);

			info.registerDeviceInterface<ArmKinematicsInterface>("armkinematics", ext_armkinematics, 0, new ArmKinematicsHandler());
		}

		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_armkinematics);
		}

	}
}
