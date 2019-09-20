/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef EXTENSION_HPP_
#define EXTENSION_HPP_

#include "Module.hpp"
#include "DeviceFactory.hpp"
#include "DeviceInterfaceT.hpp"

namespace RPI
{
	/**
	 * \brief Data class for extension creation facilities
	 */
	class RTT_EXPORT ExtensionLoaderInfo
	{
		friend class ExtensionLoader;
	private:
		ModuleFactoryMap* moduleFactory; ///< Pointer to module factory map
		DeviceFactoryMap* deviceFactory; ///< Pointer to device factory map
		DeviceInterfaceMap* interfaceMap; ///< Pointer to device interface map

	public:

		template<class T>
		void registerDeviceFactory(const std::string& devicename, const std::string& extension,
				const HTTPHandlerList& handlers)
		{
			// do not touch existing module factory
			if (deviceFactory->find(devicename) != deviceFactory->end())
				return;

			DeviceFactory* newfactory = new DeviceFactoryT<T>(handlers, interfaceMap);
			if (newfactory)
			{
				newfactory->creatorExtension = extension;
				(*deviceFactory)[devicename] = newfactory;
			}

		}

		template<class T>
		void registerModuleFactory(const std::string& modulename, const std::string& extension)
		{
			// do not touch existing module factory
			if (moduleFactory->find(modulename) != moduleFactory->end())
				return;

			ModuleFactory* newfactory = new ModuleFactoryT<T>();
			if (newfactory)
			{
				newfactory->creatorExtension = extension;
				(*moduleFactory)[modulename] = newfactory;
			}

		}

		template<class T>
		void registerDeviceInterface(const std::string& interfacename, const std::string& extension,
				std::map<std::string,std::string> (*getter)(T*), HTTPHandler* handler)
		{
			HTTPHandlerList handlers;
			handlers.push_back(HTTPHandlerInfo("", handler));

			registerDeviceInterface<T>(interfacename, extension, getter, handlers);
		}

		template<class T>
		void registerDeviceInterface(const std::string& interfacename, const std::string& extension,
				std::map<std::string,std::string> (*getter)(T*), const HTTPHandlerList& handlers)
		{
			// do not touch existing module factory
			if (interfaceMap->find(interfacename) != interfaceMap->end())
				return;

			DeviceInterface* newinterface = new DeviceInterfaceT<T>(interfacename, getter, handlers);
			if (newinterface)
			{
				newinterface->creatorExtension = extension;
				(*interfaceMap)[interfacename] = newinterface;
			}
		}

		void unregisterExtension(const std::string& extension)
		{
			// Remove modules
			for (ModuleFactoryMap::iterator it = moduleFactory->begin(); it != moduleFactory->end();)
			{
				// Only remove those factories created by this extension
				if (it->second->creatorExtension == extension)
				{
					delete it->second;
					moduleFactory->erase(it++);
				} else
				{
					++it;
				}
			}

			// Remove devices
			for (DeviceFactoryMap::iterator it = deviceFactory->begin(); it != deviceFactory->end();)
			{
				// Only remove those factories created by this extension
				if (it->second->creatorExtension == extension)
				{
					it->second->cleanupHTTPHandlers();

					delete it->second;
					deviceFactory->erase(it++);
				} else
				{
					++it;
				}
			}

			// Remove DeviceInterfaces
			for (DeviceInterfaceMap::iterator it = interfaceMap->begin(); it != interfaceMap->end();)
			{
				// Only remove those factories created by this extension
				if (it->second->creatorExtension == extension)
				{
					delete it->second;
					interfaceMap->erase(it++);
				} else
				{
					++it;
				}
			}

		}
	};
}

#endif /* EXTENSION_HPP_ */
