/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef NETFWD_HPP_
#define NETFWD_HPP_

#include <map>
#include <string>
#include <rtt/os/TimeService.hpp>

#include "ModuleFwd.hpp"

namespace RPI
{
	typedef int Net_ID_t;

	const Net_ID_t NET_INVALID = -1;

	typedef int Session_ID_t;
	const Session_ID_t SESSION_INVALID = -1;
	const Session_ID_t SESSION_NONE = 0;

	typedef std::map<std::string, std::pair<std::string, RTT::os::TimeService::nsecs> > Netcomm_Map_t;

	typedef std::map<std::string, std::pair<ModuleFactory*, Module*> > taskmap_t;

	enum NetState
	{
		Loading, Rejected, Ready, Running, Terminated, Cancelling, Invalid, Scheduled, Unloading
	};


	enum NetErrorMajor
	{
		NoError, ConfigurationError, StartError, RuntimeError, InternalFail
	};

	enum NetConfigurationError
	{
		DuplicateCancel, // Deprecated
		DuplicateTermination, // Deprecated
		InvalidModuleType,
		ModuleLoadingFailed,
		NoTermination,
		InvalidProperty,
		ModuleConfigurationFailed,
		NetCycle,
		InvalidInport,
		InvalidSourceModule,
		InvalidSourceOutport,
		PortConnectionFailed,
		XMLParsingFailed,
		DuplicateTakeover, // Deprecated
		ParentNetAborted,
		DeviceNotFound
	};

	class NetErrorState;

	class Net;

}



#endif /* NETFWD_HPP_ */
