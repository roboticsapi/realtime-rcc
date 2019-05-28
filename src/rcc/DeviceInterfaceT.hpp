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
	public:
		DeviceInterfaceT(const std::string& name, const HTTPHandlerList& handlers) :
				DeviceInterface(name, handlers)
		{

		}

		bool checkDeviceInterface(Device* device)
		{
			T* testdevice = dynamic_cast<T*>(device);

			return testdevice != 0;
		}
	};
}

#endif /* DEVICEINTERFACET_HPP_ */
