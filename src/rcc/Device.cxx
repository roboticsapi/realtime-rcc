/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "Device.hpp"
#include "CrashDump.hpp"
#include "CrashDumper.hpp"

using namespace std;

namespace RPI
{
	Device::Device(std::string name, const parameter_t& initialparameters)
	{
		parameters = initialparameters;
		this->name = name;
		crashDumpRequested = false;
	}

	Device::~Device()
	{
		// TODO Auto-generated destructor stub
	}

	void Device::setParameter(string const& name, string const& value)
	{
		set<string> mutablep = getMutableParameters();
		string lowername = boost::to_lower_copy(name);
		if (mutablep.find(lowername) != mutablep.end())
		{
			parameters[lowername] = value;
		}
	}

	string Device::getParameter(string const& name, const string& defaultvalue) const
	{
		string lowername = boost::to_lower_copy(name);
		parameter_t::const_iterator it;
		it = parameters.find(lowername);
		if (it != parameters.end())
		{
			return it->second;
		}
		return defaultvalue;
	}

	string Device::getName() const
	{
		return name;
	}

	set<string> Device::getParameters() const
	{
		set<string> result;

		for(parameter_t::const_iterator it = parameters.begin(); it != parameters.end(); ++it)
		{
			result.insert(it->first);
		}

		return result;
	}

	void Device::lockDevice()
	{

	}

	void Device::unlockDevice()
	{

	}

	void Device::merge_default_parameters(parameter_t& parameters, const parameter_t& defaultparameters)
	{
		for(parameter_t::const_iterator it = defaultparameters.begin(); it != defaultparameters.end(); ++it)
		{
			if(parameters.find(it->first) == parameters.end())
			{
				parameters[it->first] = it->second;
			}
		}
	}

	void Device::addCrashDumper(CrashDumper* crashDumper) {
		crashDumpers.push_back(crashDumper);
	}

	void Device::triggerCrashDump()
	{
		for(const auto& dumper: crashDumpers)
			dumper->freeze();
		crashDumpRequested = true;
	}

	void Device::processCrashDump()
	{
		if(crashDumpRequested) {
			CrashDump::writeCrashDump(name, crashDumpers);
			crashDumpRequested = false;
		}
	}

	const HTTPHandlerList& DeviceFactory::getHTTPHandlers() const
	{
		return httpHandlers;
	}

	DeviceFactory::DeviceFactory(const HTTPHandlerList& handlers)
	{
		httpHandlers = handlers;
	}

	DeviceFactory::~DeviceFactory()
	{

	}
}
