/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DEVICEFACTORY_HPP_
#define DEVICEFACTORY_HPP_

#include "Device.hpp"
#include "DeviceInterface.hpp"
#include "Registry.hpp"
#include "Server/HTTPServer.hpp"

namespace RPI
{
	/**
	 * \brief Template implementation for DeviceFactory
	 */
	template<class C> class DeviceFactoryT: public DeviceFactory
	{
	public:
		DeviceFactoryT(const HTTPHandlerList& handlers, DeviceInterfaceMap* deviceinterfaces) :
				DeviceFactory(handlers), deviceInterfaces(deviceinterfaces)
		{
		}

		Device* createInstance(const std::string& name, const parameter_t& parameter)
		{
			Device* device = C::createDevice(name, parameter);

			const HTTPHandlerList& handlers = getHTTPHandlers();

			for (HTTPHandlerList::const_iterator it = handlers.begin(); it != handlers.end(); ++it)
			{
				HTTPHandlerInfo info = *it;
				std::string p = "/devices/" + name + info.path;
				HTTPServer::getInstance()->addHandler(p, info.handler);
			}

			updateInterfaces(name, device);

			return device;
		}

		void updateInterfaces(const std::string& name, Device* device)
		{
			// Check for DeviceInterfaces
			for(DeviceInterfaceMap::const_iterator it = deviceInterfaces->begin(); it != deviceInterfaces->end();++it)
			{
				if(it->second->checkDeviceInterface(device))
				{
					HTTPHandlerList interfacehandlers = it->second->getHTTPHandlers();

					for (HTTPHandlerList::const_iterator it2 = interfacehandlers.begin(); it2 != interfacehandlers.end(); ++it2)
					{
						HTTPHandlerInfo info = *it2;
						std::string p = "/devices/" + name + "/" + it->first + info.path;
						HTTPServer::getInstance()->addHandler(p, info.handler);
					}

					device->implementedDeviceInterfaces[it->first] = it->second->getDeviceParameters(device);
					device->lastUpdated = RTT::os::TimeService::Instance()->getNSecs();
				}
			}
		}

		bool destroyInstance(const std::string& name, Device* device)
		{
			// remove HTTP Handlers
			const HTTPHandlerList& handlers = getHTTPHandlers();

			for (HTTPHandlerList::const_iterator it = handlers.begin(); it != handlers.end(); ++it)
			{
				HTTPHandlerInfo info = *it;
				std::string p = "/devices/" + name + info.path;
				HTTPServer::getInstance()->removeHandler(p);
			}

			// Check for DeviceInterfaces
			for(DeviceInterfaceMap::const_iterator it = deviceInterfaces->begin(); it != deviceInterfaces->end();++it)
			{
				if(it->second->checkDeviceInterface(device))
				{
					HTTPHandlerList interfacehandlers = it->second->getHTTPHandlers();

					for (HTTPHandlerList::const_iterator it2 = interfacehandlers.begin(); it2 != interfacehandlers.end(); ++it2)
					{
						HTTPHandlerInfo info = *it2;
						std::string p = "/devices/" + name + "/" + it->first + info.path;
						HTTPServer::getInstance()->removeHandler(p);
					}

				}
			}

			delete device;

			return true;
		}

		// Method for extensions to remove all httphandlers they created
		void cleanupHTTPHandlers()
		{
			const HTTPHandlerList& handlers = getHTTPHandlers();
			for (HTTPHandlerList::const_iterator it = handlers.begin(); it != handlers.end(); ++it)
			{
				delete (*it).handler;
			}
		}
	private:
		DeviceInterfaceMap* deviceInterfaces;
	};
}

#endif /* DEVICEFACTORY_HPP_ */
