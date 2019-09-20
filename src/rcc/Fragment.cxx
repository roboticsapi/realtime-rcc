/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "Fragment.hpp"
#include "Net.hpp"
#include "NetModules.hpp"
#include "Registry.hpp"
#include "VirtualRCCDevice.hpp"
#include <queue>
#include <iostream>

namespace RPI
{

	Fragment::Fragment(Net* net, RPI::ModuleFactoryMap* factory, rpi_fragment fragment) :
			RPI::ActiveModule(fragment.id, net), modules(), net(net), factory(factory), fragment(fragment)
	{

	}

	Fragment::~Fragment()
	{
		for (std::list<Fragment*>::iterator it = loaded_fragments.begin(); it != loaded_fragments.end(); ++it)
		{
			delete *it;
		}

		for (std::list<std::pair<ModuleFactory*, Module*> >::iterator it = loaded_modules.begin();
				it != loaded_modules.end(); ++it)
				{
			ModuleFactory* mf = it->first;
			if (mf)
				mf->destroyInstance(it->second);
			else
				delete it->second;
		}
		loaded_fragments.clear();
		loaded_modules.clear();
		modules.clear();
	}

	bool Fragment::buildFragment()
	{
		modulemap_t moduleMap;
		topomap_t leftMap, rightMap;
		std::queue<std::string> empty;
		leftMap[fragment.id] = topomapitem_t();
		rightMap[fragment.id] = topomapitem_t();

		// create fragments
		for (std::list<rpi_fragment>::iterator it = fragment.fragments.begin(); it != fragment.fragments.end(); ++it)
		{
			Fragment *fragment = new Fragment(net, factory, *it);
			loaded_fragments.push_back(fragment);

			if (!fragment || !fragment->buildFragment())
				return false;

			moduleMap[it->id] = fragment;
			leftMap[it->id] = topomapitem_t();
			rightMap[it->id] = topomapitem_t();
		}

		for (std::list<rpi_module>::iterator it = fragment.modules.begin(); it != fragment.modules.end(); ++it)
		{
			ModuleFactory* mf;

			ModuleFactoryMap::const_iterator mfit = factory->find(it->type);

			if (mfit == factory->end())
			{
				net->configurationError(InvalidModuleType, "Could not find module of type " + it->type);
				return false;
			} else {
				mf = mfit->second;
			}

			Module *module = (*factory)[it->type]->createInstance(it->id, net);

			if (!module)
			{
				net->configurationError(ModuleLoadingFailed, "Could not instantiate module " + it->id);
				return false;
			}

			loaded_modules.push_back(std::pair<ModuleFactory*, Module*>(mf, module));

			moduleMap[it->id] = module;
			leftMap[it->id] = topomapitem_t();
			rightMap[it->id] = topomapitem_t();
			for (std::list<rpi_parameter>::iterator param = it->parameters.begin(); param != it->parameters.end();
					++param)
					{
				PropertyBase *prop = module->properties()->find(param->name);
				if (!prop)
				{
					net->configurationError(InvalidProperty,
							"Error: Property " + param->name + " not found in module " + it->id, it->id);
					return false;
				}
				module->setPropertyValue(prop, param->value);
			}
		}

		// connect ports
		for (std::list<rpi_fragment>::iterator it = fragment.fragments.begin(); it != fragment.fragments.end(); ++it)
		{
			if (!connectPorts(it, moduleMap, leftMap, rightMap, empty))
				return false;
		}

		for (std::list<rpi_module>::iterator it = fragment.modules.begin(); it != fragment.modules.end(); ++it)
		{
			if (!connectPorts(it, moduleMap, leftMap, rightMap, empty))
				return false;
		}

		// create out ports
		for (std::list<rpi_port>::iterator it = fragment.outPorts.begin(); it != fragment.outPorts.end(); ++it)
		{
			Module* srcModule = moduleMap[it->from_module];
			if (!srcModule)
			{
				inNet->configurationError(InvalidSourceModule,
						"Fragment " + fragment.id + ": source primitive " + it->from_module + " does not exist",
						fragment.id);

				return false;
			}

			Port* srcPort = srcModule->ports()->getSinglePort(it->from_port);

			if (!srcPort)
			{
				inNet->configurationError(InvalidSourceOutport,
						"Module " + it->from_module + " has no outPort " + it->from_port, fragment.id);

				return false;
			}

			ports()->addNamedPort(it->name, srcPort);
		}

		// find actuator and sensor primitives
		for(const auto& it : moduleMap)
		{
			if(it.second->isSensor())
				sensorModules.push_back(it.second);
			if(it.second->isActuator())
				actuatorModules.push_back(it.second);
		}

		// topologically sort net
		for (std::map<std::string, Module*>::iterator it = moduleMap.begin(); it != moduleMap.end(); ++it)
		{
			rightMap[it->first].unique();
		}

		while (!moduleMap.empty())
		{
			if (empty.empty())
			{
				std::stringstream message;
				message << "Cycle found in net\n";
				for (std::map<std::string, Module*>::iterator it = moduleMap.begin(); it != moduleMap.end(); ++it)
				{
					message << "Module in cycle: " << it->first << "\n";
				}
				std::cout << message.str() << std::endl;

				net->configurationError(NetCycle, message.str());
				return false;
			} else
			{
				std::string i = empty.front();
				empty.pop();

				modulemap_t::iterator mit = moduleMap.find(i);

				if (mit != moduleMap.end())
				{
					modules.push_back(mit->second);
					moduleMap.erase(mit);

					for (std::list<std::string>::iterator it = rightMap[i].begin(); it != rightMap[i].end(); ++it)
					{
						leftMap[*it].remove(i);
						if (leftMap[*it].size() == 0)
							empty.push(*it);
					}
				}
			}
		}

		return true;

	}

	template<class T>
	bool Fragment::connectPorts(T& it, modulemap_t& moduleMap, topomap_t& leftMap, topomap_t& rightMap,
			std::queue<std::string>& empty)
	{
		int debugLevel = 1;
		double debugTime = 2.0;

		if (const VirtualRCCDevice* vrcc = Registry::getRegistry()->getVirtualRCC())
		{
			debugLevel = vrcc->getDebuggingLevel();
			debugTime = vrcc->getDebuggingTime();
		}
		Module *destModule = moduleMap[it->id];

		for (std::list<rpi_port>::iterator port = it->inPorts.begin(); port != it->inPorts.end(); ++port)
		{
			if (!connectPort(&(*port), destModule, it->id, moduleMap, leftMap, rightMap))
				return false;

			// add debug port if necessary
			if (debugLevel == 2 || (debugLevel == 1 && port->debug > 0))
			{
				TypeKit* type = (dynamic_cast<Port*>(destModule->ports()->getFirstPort(port->name)))->getType();
				std::string debugname = it->id + "." + port->name;
				int debugsize;

				if(debugLevel == 2)
					debugsize = debugTime / inNet->getNetFrequency();
				else
					debugsize = (port->debug > 10) ? 10 / inNet->getNetFrequency() : port->debug / inNet->getNetFrequency();

				DebugModule* mod = type->createDebug(debugname, net, debugsize);

				if (mod)
				{
					moduleMap[debugname] = mod;
					leftMap[debugname] = topomapitem_t();
					rightMap[debugname] = topomapitem_t();

					rpi_port debugport = *port;
					debugport.name = "inValue";
					debugport.debug = 0;

					if (!connectPort(&debugport, mod, debugname, moduleMap, leftMap, rightMap))
						return false;

					if (leftMap[debugname].size() == 0)
					{
						empty.push(debugname);
					}
				}
			}

		}

		if (leftMap[it->id].size() == 0)
		{
			empty.push(it->id);
		}
		return true;
	}

	bool Fragment::connectPort(rpi_port* port, Module* destModule, const std::string& modulename,
			modulemap_t& moduleMap, topomap_t& leftMap, topomap_t& rightMap)
	{
		std::list<Port*> destPorts = destModule->ports()->getPort(port->name);

		if (destPorts.empty())
		{
			inNet->configurationError(InvalidInport, "Module " + destModule->getName() + " has no inPort " + port->name,
					destModule->getName());
			return false;
		}

		if (port->from_module == fragment.id)
		{
			for (std::list<Port*>::iterator it = destPorts.begin(); it != destPorts.end(); ++it)
				ports()->addNamedPort(port->from_port, *it);
		} else
		{
			Module *srcModule = moduleMap[port->from_module];

			if (!srcModule)
			{
				inNet->configurationError(
						InvalidSourceModule,
						"Module " + destModule->getName() + ": source primitive " + port->from_module
								+ " does not exist", destModule->getName());
				return false;
			}

			Port *srcPort = srcModule->ports()->getSinglePort(port->from_port);

			if (!srcPort)
			{
				inNet->configurationError(InvalidSourceOutport,
						"Module " + port->from_module + " has no outPort " + port->from_port, destModule->getName());
				return false;
			}

			for (std::list<Port*>::iterator it = destPorts.begin(); it != destPorts.end(); ++it)
			{
				if (!(*it)->connectWith(srcPort))
				{
					inNet->configurationError(PortConnectionFailed, "Could not connect ports", destModule->getName());
					return false;
				}

				if (!(*it)->ready())
				{
					inNet->configurationError(
							PortConnectionFailed,
							"Module " + destModule->getName() + " inPort " + port->name
									+ " did not connect properly. Check types!", destModule->getName());
					return false;
				}
			}

			if (!srcPort->getModule()->isPre())
			{
				leftMap[modulename].push_back(port->from_module);
				rightMap[port->from_module].push_back(modulename);
			}
		}
		return true;
	}

	bool Fragment::configureHook()
	{
		for (std::list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			if (!(*it)->configure())
			{
				net->configurationError(ModuleConfigurationFailed,
						"Configuration of module " + (*it)->getName() + " failed", (*it)->getName());
				return false;
			}
		}
		return true;
	}

	bool Fragment::startHook()
	{
		bool ret = true;
		for (std::list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			ret &= (*it)->start();
		}
		return ret;
	}

	void Fragment::updateHook()
	{
		if (active())
		{
			for (std::list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
			{
				(*it)->updateHook();
			}
		}
	}

	void Fragment::updatePre()
	{
		for (std::list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			(*it)->updatePre();
		}
	}

	void Fragment::stopHook()
	{
		for (std::list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			(*it)->stop();
		}
	}

	void Fragment::cleanupHook()
	{
		for (std::list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			(*it)->cleanup();
		}
	}

	std::vector<Module*> Fragment::getLeafModules()
	{
		std::vector<Module*> ret;
		for (std::list<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			std::vector<Module*> leaves = (*it)->getLeafModules();
			for (std::vector<Module*>::iterator li = leaves.begin(); li != leaves.end(); ++li)
			{
				ret.push_back(*li);
			}
		}
		return ret;
	}

	std::set<std::string> Fragment::getResourceNames() const
	{
		std::set<std::string> result, tmp;
		for (std::list<Module*>::const_iterator it = modules.begin(); it != modules.end(); ++it)
		{
			tmp = (*it)->getResourceNames();
			result.insert(tmp.begin(), tmp.end());
		}

		return result;
	}

	void Fragment::updateSensor()
	{
		// must not check for isActive - data flow will still represent old state
		for (const auto& sensor : sensorModules)
			sensor->updateSensor();
	}

	void Fragment::updateActuator()
	{
		if (active())
			for (const auto& actuator : actuatorModules)
				actuator->updateActuator();
	}

	// Check whether at least one primitive is sensor
	bool Fragment::isSensor()
	{
		return sensorModules.size() > 0;
	}

	// Check whether at least one primitive is actuator
	bool Fragment::isActuator()
	{
		return actuatorModules.size() > 0;
	}

}
