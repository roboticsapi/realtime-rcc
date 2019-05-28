/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "Module.hpp"
#include "Net.hpp"
#include "DeviceInstanceT.hpp"

using namespace std;

namespace RPI
{

	Module::Module(const std::string& name, Net* net) :
		ModuleBase(name, net), resourceUsed(0), resourceCount(0), minExecutionTime(RTT::os::TimeService::InfiniteNSecs),
				maxExecutionTime(0), totalExecutionTime(0)
	{
		inNet = net;
		resourceNamesCacheOk = false;
	}

	/**
	 * clean up resource pointers
	 */
	Module::~Module()
	{
		if (resourceCount > 0)
			delete[] resourceUsed;
	}

	void Module::allocateResourceArray(int size)
	{
		if (size > 0)
		{
			resourceUsed = new bool*[size];
			resourceCount = size;
		}
	}

	set<string> Module::getResourceNames() const
	{
		set<string> ret;
		return ret;
	}

	const set<string>& Module::getCachedResourceNames() const
	{
		if(!resourceNamesCacheOk)
		{
			resourceNamesCache = getResourceNames();
			resourceNamesCacheOk = true;
		}
		return resourceNamesCache;
	}

	bool Module::portConnected(Port* port)
	{
		if (!port->connected())
		{
			if(inNet != 0) {
				inNet->configurationError(ModuleConfigurationFailed, "The required port " + port->getName() + " is not connected.", getName());
			}
			return false;
		}
		else
		{
			return true;
		}
	}

	bool Module::deviceAvailable(DeviceInstance* devins, std::string deviceId)
	{
		if(devins->fetchInstance(deviceId) == 0) {
			inNet->configurationError(ModuleConfigurationFailed, "The required device " + deviceId + " is not available.", getName());
			return false;
		}
		else
		{
			return true;
		}
	}


	bool Module::checkResources() const
	{
		bool success = true;
		const set<string>& resources = getCachedResourceNames();

		for (set<string>::const_iterator it = resources.begin(); it != resources.end(); ++it)
		{
			string resourceUsage = ResourceManager::getResourceManager()->getResource(*it);
			success &= ((resourceUsage == "") || (resourceUsage == inNet->getName()));
		}
		return success;
	}

	/**
	 * Registers the module at the resource handler for the current net
	 * aquireResource must be called under the same Mutex as checkResource to avoid Resources been taken
	 * by other threads after checking
	 */
	void Module::acquireResources()
	{
		const set < string >& resources = getCachedResourceNames();
		for (set<string>::const_iterator it = resources.begin(); it != resources.end(); ++it)
		{
			ResourceManager::getResourceManager()->addResource(*it, inNet->getName());
		}
	}

	/**
	 * Unregisters the module at the resource handler
	 */
	void Module::releaseResources()
	{
		const set < string >& resources = getCachedResourceNames();
		for (set<string>::const_iterator it = resources.begin(); it != resources.end(); ++it)
		{
			ResourceManager::getResourceManager()->removeResource(*it);
		}
	}

	const Net* Module::getInNet() const
	{
		return inNet;
	}

	long Module::getNetCurrentCycle() const
	{
		if (getInNet())
			return getInNet()->getCurrentCycle();
		return 0;
	}


	std::vector<Module*> Module::getLeafModules() {
		std::vector<Module*> ret;
		ret.push_back(this);
		return ret;
	}

	ActiveModule::ActiveModule(const string& name, Net* net) :
		Module(name, net), inActive("inActive", this)
	{
		this->ports()->addPort(&inActive, "Activation port");
	}

	/**
	 * clean up resource pointers
	 */
	ActiveModule::~ActiveModule()
	{

	}

	StatefulModule::StatefulModule(const string& name, Net* net) :
		ActiveModule(name, net), inReset("inReset", this)
	{
		this->ports()->addPort(&inReset, "Reset port");
	}

	/**
	 * clean up resource pointers
	 */
	StatefulModule::~StatefulModule()
	{

	}

	string ResourceManager::getResource(const string& resourceID) const
	{
		map<string, string>::const_iterator it = resourceMap.find(resourceID);
		if(it != resourceMap.end())
			return it->second;
		else
			return "";
	}

	void ResourceManager::addResource(const string& resourceID, const string& netID)
	{
		resourceMap[resourceID] = netID;
	}

	void ResourceManager::removeResource(const string& resourceID)
	{
		resourceMap.erase(resourceID);
	}

	/**
	 * remove all locks for specific net (for use in ~Net)
	 */
	void ResourceManager::removeNetUsage(const string& netID)
	{
		set < string > toremove;
		for (map<string, string>::iterator it = resourceMap.begin(); it != resourceMap.end(); ++it)
			if (it->second == netID)
				toremove.insert(it->first);

		for (set<string>::iterator it = toremove.begin(); it != toremove.end(); ++it)
			resourceMap.erase(*it);

	}

	ResourceManager* ResourceManager::theManager = 0;

	ResourceManager::ResourceManager()
	{
	}

	ResourceManager* ResourceManager::getResourceManager()
	{
		if (!theManager)
			theManager = new ResourceManager();
		return theManager;
	}

	ModuleFactory::ModuleFactory()
	{

	}

	ModuleFactory::~ModuleFactory()
	{

	}
}
