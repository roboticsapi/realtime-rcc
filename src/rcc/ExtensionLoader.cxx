/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef _WIN32
#include <dlfcn.h>
#endif

#include <boost/algorithm/string.hpp>
#include <rtt/Logger.hpp>

#include <boost/filesystem.hpp>
#include <boost/asio/ip/host_name.hpp>

#include "rapidxml/rapidxml_utils.hpp"
#include "rapidxml/rpixmlutil.hpp"

#include "Server/HTTPHandlers.hpp"
#include "ExtensionLoader.hpp"
#include "NetModules.hpp"
#include "Registry.hpp"

#include "version_bin.h"

namespace RPI
{
	using namespace rapidxml;
	using namespace std;

	ExtensionLoader::ExtensionLoader() :
			moduleFactoryMap(), deviceFactoryMap(), extensionLoaderMap()
	{
		loaderInfo.moduleFactory = &moduleFactoryMap;
		loaderInfo.deviceFactory = &deviceFactoryMap;
		loaderInfo.interfaceMap = &deviceInterfaceMap;

		loadInternalModules();
	}

	ExtensionLoader::~ExtensionLoader()
	{
		unloadInternalModules();

		// unload all modules still found in moduleLoaderMap
		for (ExtensionLoaderMap::iterator it = extensionLoaderMap.begin(); it != extensionLoaderMap.end(); ++it)
		{
			rccext_unload_t unload = it->second->unloader;
			unload(loaderInfo);
			dlclose(it->second->dl);

			delete it->second;
		}

	}

	void ExtensionLoader::loadExtensions(const string& filename)
	{
		try
		{
			rapidxml::file<> xmlfile(filename.c_str());

			xml_document<> doc;
			doc.parse<0>(xmlfile.data());

			xml_node<>* rootnode = doc.first_node();

			if (!rootnode)
			{
				RTT::log(RTT::Error) << "Root tag not found in " << filename << RTT::endlog();
				return;
			}

			for (xml_node<>* node = rootnode->first_node("Extension"); node; node = node->next_sibling("Extension"))
			{
				xml_attribute<>* attr = node->first_attribute("file");

				if (!attr)
				{
					RTT::log(RTT::Error) << "Ill-formed Extension tag in " << filename << RTT::endlog();
				} else
				{
					string modulefilename(attr->value());
					if (modulefilename.compare(""))
						loadExtensionFile(modulefilename);
				}
			}

		} catch (...)
		{
			RTT::log(RTT::Error) << "Could not read " << filename << RTT::endlog();
		}
	}

	void ExtensionLoader::loadDevices(const string& filename)
	{
		xml_document<> doc;

		configuration_t config;
		try
		{
			rapidxml::file<> xmlfile(filename.c_str());

			xml_document<> doc;
			doc.parse<0>(xmlfile.data());

			xml_node<>* rootnode = doc.first_node();

			if (!rootnode)
			{
				RTT::log(RTT::Error) << "Root tag not found in " << filename << RTT::endlog();
				return;
			}

			config = loadDeviceConfiguration(rootnode);

			for (configuration_t::iterator it = config.begin(); it != config.end(); ++it)
			{
				string name = it->name;
				string type = it->type;
				parameter_t parameters = it->parameters;

				if (!Registry::getRegistry()->createDevice(type, name, parameters))
				{
					RTT::log(RTT::Critical) << "Could not create device " << name << " of type " << type
							<< RTT::endlog();
				}
			}

		} catch (...)
		{
			RTT::log(RTT::Error) << "Could not read " << filename << RTT::endlog();
		}
	}

	ModuleFactoryMap* ExtensionLoader::getModuleFactoryMap()
	{
		return &moduleFactoryMap;
	}

	DeviceFactoryMap* ExtensionLoader::getDeviceFactoryMap()
	{
		return &deviceFactoryMap;
	}

	ExtensionLoaderMap* ExtensionLoader::getExtensionLoaderMap()
	{
		return &extensionLoaderMap;
	}

	bool ExtensionLoader::loadExtensionFile(const string& module)
	{
		string filename;
#ifdef _WIN32
#if _DEBUG
		filename = module + "d.dll";
#else
		filename = module + ".dll";
#endif
#else
		filename = "./lib" + module + ".so";
#endif

		void* dl = dlopen(filename.c_str(), RTLD_NOW | RTLD_GLOBAL);
		if (!dl)
		{
			RTT::log(RTT::Error) << "Cannot dlopen " << dlerror() << RTT::endlog();
			return false;
		}

		try
		{
			rccext_version_t version = (rccext_version_t) dlsym(dl, "version");
			if (!version)
			{
				RTT::log(RTT::Error) << "Extension " << module
						<< " does not contain a version ID, probably outdated extension library" << RTT::endlog();
				throw ExtensionLoaderException();
			}

			rcc_bin_version_t loaded_version = version();
			if (loaded_version != rcc_bin_version_id)
			{
				RTT::log(RTT::Error) << "Extension " << module << " has version ID 0x" << std::hex << loaded_version
						<< ", RCC binary expects 0x" << rcc_bin_version_id << std::dec <<", Probably outdated extension library"
						<< RTT::endlog();
				throw ExtensionLoaderException();
			}

			rccext_load_t load = (rccext_load_t) dlsym(dl, "load");
			if (!load)
			{
				RTT::log(RTT::Error) << "Invalid extension " << module << dlerror() << RTT::endlog();
				throw ExtensionLoaderException();
			}

			rccext_unload_t unload = (rccext_unload_t) dlsym(dl, "unload");
			if (!unload)
			{
				RTT::log(RTT::Error) << "Invalid extension " << module << dlerror() << RTT::endlog();
				throw ExtensionLoaderException();
			}

			rccext_loader_t* loadert = new rccext_loader_t();
			loadert->loader = load;
			loadert->unloader = unload;
			loadert->dl = dl;
			loadert->filename = filename;

			extensionLoaderMap[module] = loadert;

			// load the module, fill moduleFactoryMap and deviceFactoryMap
			load(loaderInfo);
		} catch (ExtensionLoaderException&)
		{
			// Loading extension failed, close file handle
			dlclose(dl);
			return false;
		}

		return true;
	}

	configuration_t ExtensionLoader::loadDeviceConfiguration(xml_node<>* node)
	{
		configuration_t devices;

		for (xml_node<>* devnode = node->first_node("Device"); devnode; devnode = devnode->next_sibling("Device"))
		{
			if (devnode->type() == node_element)
			{
				ConfDevice device;
				string devicename;

				device.name = getAttribute(devnode, "name");
				device.type = boost::to_lower_copy(getAttribute(devnode, "type"));

				for (xml_node<>* parameternode = devnode->first_node("Parameter"); parameternode; parameternode = parameternode->next_sibling("Parameter"))
				{
					string parametername, parametervalue;

					if (parameternode->type() == node_element)
					{
						parametername = boost::to_lower_copy(getAttribute(parameternode, "name"));

						char* cparametervalue = parameternode->value();

						//parameter.value = RPIXMLUtil::TrimString(parametervalue, " \t\r\n");
						parametervalue = cparametervalue;
						boost::trim(parametervalue);

						device.parameters[parametername] = parametervalue;
					}
				}
				devices.push_back(device);
			}
		}
		return devices;
	}

	// methods for Singleton creation
	ExtensionLoader* ExtensionLoader::theInstance = 0;

	/**
	 * Get instance of the RPIModuleLoader class
	 * A new instance is dynamically created if has none been created yet
	 */
	ExtensionLoader* ExtensionLoader::getInstance()
	{
		if (!theInstance)
		{
			theInstance = new ExtensionLoader();
		}
		return theInstance;
	}

	void ExtensionLoader::loadInternalModules()
	{
		loaderInfo.registerModuleFactory<Cancel>("Cancel", "internal");
		loaderInfo.registerModuleFactory<Takeover>("Takeover", "internal");
		loaderInfo.registerModuleFactory<EStop>("EStop", "internal");
	}

	void ExtensionLoader::unloadInternalModules()
	{
		loaderInfo.unregisterExtension("internal");
	}

	/**
	 * Destroys the current instance of the RPIModuleLoader. The next call of
	 * getInstance() will automatically generate a new instance (with no loaded
	 * modules)
	 */
	void ExtensionLoader::finalize()
	{
		delete theInstance;
		theInstance = 0;
	}

	void ExtensionLoader::loadConfD()
	{
		using namespace boost::filesystem;

		try
		{
			string hostname = boost::asio::ip::host_name();

			path pconfd("conf.d/" + hostname);

			if (!is_directory(pconfd))
			{
				pconfd = path("conf.d");
				RTT::log(RTT::Info) << "Using main conf.d directory" << RTT::endlog();
			} else
			{
				RTT::log(RTT::Info) << "Using conf.d/" << hostname << " configuration directory" << RTT::endlog();
			}

			vector<path> pcontents;

			vector<string> ext_files, dev_files;

			copy(directory_iterator(pconfd), directory_iterator(), back_inserter(pcontents));

			sort(pcontents.begin(), pcontents.end());

			for (auto it(pcontents.begin()); it != pcontents.end(); ++it)
			{
				if (is_regular_file(*it))
				{
					const string nat = it->generic_string();
					if (nat.find("extensions.xml") != string::npos)
					{
						ext_files.push_back(nat);
					} else if (nat.find("devices.xml") != string::npos)
					{
						dev_files.push_back(nat);
					}
				}
			}

			for (const auto& it : ext_files)
			{
				loadExtensions(it);
			}

			for (const auto& it : dev_files)
			{
				loadDevices(it);
			}
		} catch (...)
		{
			RTT::log(RTT::Critical) << "No configuration directory found!" << RTT::endlog();
		}
	}

}
