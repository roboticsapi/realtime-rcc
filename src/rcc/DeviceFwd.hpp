/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DEVICEFWD_HPP_
#define DEVICEFWD_HPP_

#include <string>
#include <map>
#include <list>
#include <vector>

namespace RPI
{
	typedef std::map<std::string, std::string> parameter_t;

	enum class DeviceState {
		OFFLINE,            // device is not connected / reachable
		SAFE_OPERATIONAL,    // device is in emergency stop mode
		OPERATIONAL         // device is up and running
	};

	/**
	 * \brief Configuration info for new devices
	 */
	struct ConfDevice
	{
		std::string name;
		std::string type;
		parameter_t parameters;
	};

	typedef std::list<ConfDevice> configuration_t;

	class Device;

	class HTTPHandlerInfo;

	typedef std::vector<HTTPHandlerInfo> HTTPHandlerList;

	class DeviceFactory;

	typedef std::map<std::string, DeviceFactory*> DeviceFactoryMap;
}


#endif /* DEVICEFWD_HPP_ */
