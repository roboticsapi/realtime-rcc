/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DEVICEINTERFACET_HPP_
#define DEVICEINTERFACET_HPP_

#include "DeviceInterface.hpp"

namespace RPI
{
	template<class T>
	class RTT_EXPORT DeviceInterfaceT: public DeviceInterface
	{
	private:
		std::map<std::string,std::string> (*getter)(T*);

	public:
		DeviceInterfaceT(const std::string& name, std::map<std::string,std::string> (*getter)(T*), const HTTPHandlerList& handlers) :
				DeviceInterface(name, handlers), getter(getter)
		{
		}

		bool checkDeviceInterface(Device* device)
		{
			T* testdevice = dynamic_cast<T*>(device);

			return testdevice != 0;
		}

		std::map<std::string,std::string> getDeviceParameters(Device* device)
		{
			if(getter == 0)
				return std::map<std::string,std::string>();
			else
				return getter(dynamic_cast<T*>(device));
		}

	};
}

#endif /* DEVICEINTERFACET_HPP_ */
