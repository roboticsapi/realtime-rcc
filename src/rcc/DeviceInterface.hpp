/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DEVICEINTERFACE_HPP_
#define DEVICEINTERFACE_HPP_

#include "Device.hpp"

namespace RPI
{
	class RTT_EXPORT DeviceInterface
	{
	public:
		DeviceInterface(const std::string& name, const HTTPHandlerList& handlers);
		virtual ~DeviceInterface();

		virtual bool checkDeviceInterface(Device* device) = 0;

		virtual std::map<std::string,std::string> getDeviceParameters(Device* device) = 0;

		HTTPHandlerList getHTTPHandlers() const;

		std::string creatorExtension;
	private:
		std::string name;
		HTTPHandlerList httpHandlers;
	};

	typedef std::map<std::string, DeviceInterface*> DeviceInterfaceMap;

} /* namespace RPI */
#endif /* DEVICEINTERFACE_HPP_ */
