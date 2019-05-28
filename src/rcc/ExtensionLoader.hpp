/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef EXTENSIONLOADER_H_
#define EXTENSIONLOADER_H_

#include "rapidxml/rapidxml.hpp"

#include "Module.hpp"

#include "Extension.hpp"
#include "version_bin.h"


namespace RPI
{
	typedef rcc_bin_version_t (*rccext_version_t)();
	typedef void (*rccext_load_t)(ExtensionLoaderInfo);
	typedef void (*rccext_unload_t)(ExtensionLoaderInfo);

	struct rccext_loader_t
	{
		rccext_load_t loader;
		rccext_unload_t unloader;
		void* dl;
		std::string filename;
	};

	typedef std::map<std::string, rccext_loader_t*> ExtensionLoaderMap;

	class ExtensionLoaderException: public std::exception
	{

	};

	/**
	 * \brief Loading facilities for extensions (shared libraries)
	 */
	class ExtensionLoader
	{
	public:
		/**
		 * \brief Singleton accessor
		 */
		static ExtensionLoader* getInstance();
		/**
		 * \brief Manually destroy singleton instance
		 *
		 * The singleton instance of ExtensionLoader can be manually destroyed using this finalize
		 * method. Calling getInstance afterwards will recreate singleton instance.
		 */
		static void finalize();

		/**
		 * \brief load all extensions from conf.d directory
		 */
		void loadConfD();

		/**
		 * \brief load extensions from XML file
		 * \param filename Filename to read
		 */
		void loadExtensions(const std::string& filename);
		/**
		 * \brief load devices from XML file
		 * \param filename Filename to read
		 */
		void loadDevices(const std::string& filename);
		/**
		 * \brief Get global map of module factories
		 */
		ModuleFactoryMap* getModuleFactoryMap();
		/**
		 * \brief Get global map of device factories
		 */
		DeviceFactoryMap* getDeviceFactoryMap();
		/**
		 * \brief Get global map of device factories
		 */
		DeviceInterfaceMap* getDeviceInterfaceMap();
		/**
		 * \brief Get global map of extension loader informations
		 */
		ExtensionLoaderMap* getExtensionLoaderMap();
		/**
		 * \brief load extension (.dll or .so file)
		 * \param module Name of extension to load, no library prefix or suffix
		 * \return true, if extension could be successfully loaded, false otherwise
		 *
		 * The specified extension is loaded and registered. The filename of the extension
		 * will be automatically constructed:
		 *
		 * - Linux
		 * 		- "lib" + module + ".so"
		 * - Windows
		 * 		- Debug-Build: module + ".d.dll"
		 * 		- Release-Build: module + ".dll"
		 */
		bool loadExtensionFile(const std::string& module);

	private:
		ExtensionLoader();
		virtual ~ExtensionLoader();

		ModuleFactoryMap moduleFactoryMap;
		DeviceFactoryMap deviceFactoryMap;
		DeviceInterfaceMap deviceInterfaceMap;
		ExtensionLoaderMap extensionLoaderMap;

		ExtensionLoaderInfo loaderInfo;

		configuration_t loadDeviceConfiguration(rapidxml::xml_node<>* node);

		/**
		 * \brief Load modules internally defined in RealtimeRCC
		 */
		void loadInternalModules();
		/**
		 * \brief Unload modules internally defined in RealtimeRCC
		 */
		void unloadInternalModules();

		static ExtensionLoader* theInstance;
	};
}

#endif /* RPIMODULELOADER_H_ */
