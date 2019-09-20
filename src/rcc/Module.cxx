/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "Module.hpp"
#include "Net.hpp"
#include "ResourceManager.hpp"
#include "DeviceInstanceT.hpp"

using namespace std;

namespace RPI
{

	Module::Module(const std::string& name, Net* net) :
		ModuleBase(name, net), resourceUsed(0), resourceCount(0), minExecutionTime(RTT::os::TimeService::InfiniteNSecs),
				maxExecutionTime(0), totalExecutionTime(0)
	{
		inNet = net;
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

	ModuleFactory::ModuleFactory()
	{

	}

	ModuleFactory::~ModuleFactory()
	{

	}
}
