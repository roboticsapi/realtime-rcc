/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "ModuleBase.hpp"
#include "Net.hpp"
#include "Module.hpp"

namespace RPI
{
	using namespace std;

	// Constructor MUST NOT dereference module, will most likely not being already constructed
	// Port
	Port::Port(const std::string& name, Module* module) :
			typekit(0)
	{
		this->name = name;
		this->inModule = module;
	}

	Port::~Port()
	{

	}

	std::string Port::getName() const
	{
		return this->name;
	}
	std::string Port::getDescription() const
	{
		return this->desc;
	}
	void Port::setDescription(const std::string& desc)
	{
		this->desc = desc;
	}
	bool Port::ready()
	{
		return connected();
	}

	long Port::getNetCurrentCycle() const
	{
		if (inModule)
			return inModule->getNetCurrentCycle();
		return 0;
	}

	bool Port::isNull() const
	{
		return getNetCurrentCycle() != getLastWriteCycle();
	}

	Module* Port::getModule() const
	{
		return inModule;
	}

	// PropertyBase
	PropertyBase::PropertyBase(const std::string& name, const std::string& desc)
	{
		this->name = name;
		this->desc = desc;
		this->typekit = 0;
	}

	PropertyBase::~PropertyBase()
	{

	}

	std::string PropertyBase::getName() const
	{
		return name;
	}
	void PropertyBase::setDescription(const std::string& desc)
	{
		this->desc = desc;
	}
	std::string PropertyBase::getDescription() const
	{
		return desc;
	}

	// PropertyInterface
	void PropertyBag::addProperty(PropertyBase* prop)
	{
		props.push_back(prop);
	}
	void PropertyBag::addProperty(PropertyBase* prop, std::string desc)
	{
		props.push_back(prop);
		prop->setDescription(desc);
	}
	PropertyBag::Properties PropertyBag::getProperties() const
	{
		return props;
	}
	PropertyBase* PropertyBag::find(std::string name) const
	{
		for (std::vector<PropertyBase*>::const_iterator it = props.begin(); it != props.end(); ++it)
		{
			if ((*it)->getName() == name)
				return *it;
		}
		return 0;
	}

	// PortInterface
	void PortInterface::addPort(Port* port, std::string desc)
	{
		addNamedPort(port->getName(), port);
		port->setDescription(desc);
	}

	void PortInterface::addPort(Port* port)
	{
		addNamedPort(port->getName(), port);
	}

	void PortInterface::addNamedPort(std::string name, Port* port)
	{
		ports.insert(pair<string, Port*>(name, port));
	}

	list<Port*> PortInterface::getPort(const std::string& name) const
	{
		list<Port*> resports;

		pair<multimap<string, Port*>::const_iterator, multimap<string, Port*>::const_iterator> ret;

		ret = ports.equal_range(name);

		for (multimap<string, Port*>::const_iterator it = ret.first; it != ret.second; ++it)
			resports.push_back(it->second);

		return resports;
	}

	/**
	 * returns a single port for given name, or 0 if no or more than 1 port exists with
	 * the given name
	 */
	Port* PortInterface::getSinglePort(const string& name) const
	{
		list<Port*> foundports = getPort(name);

		if (foundports.size() == 1)
			return foundports.back();

		return 0;
	}

	/**
	 * returns the first port for given name, or 0 if no port exists with
	 * the given name
	 */
	Port* PortInterface::getFirstPort(const string& name) const
	{
		list<Port*> foundports = getPort(name);

		if (foundports.size() >= 1)
			return foundports.back();

		return 0;
	}

	std::vector<std::string> PortInterface::getPortNames() const
	{
		std::vector<std::string> ret;
		for (std::multimap<std::string, Port*>::const_iterator it = ports.begin(); it != ports.end(); ++it)
		{
			ret.push_back(it->first);
		}
		return ret;
	}

	std::vector<Port*> PortInterface::getPorts() const
	{
		std::vector<Port*> ret;
		for (std::multimap<std::string, Port*>::const_iterator it = ports.begin(); it != ports.end(); ++it)
		{
			ret.push_back(it->second);
		}
		return ret;
	}

	// ModuleBase
	ModuleBase::ModuleBase(const std::string& name, Net*)
	{
		this->name = name;
	}
	ModuleBase::~ModuleBase()
	{

	}
	string ModuleBase::getName() const
	{
		return this->name;
	}
	PortInterface* ModuleBase::ports()
	{
		return &this->portInterface;
	}
	PropertyBag* ModuleBase::properties()
	{
		return &this->propInterface;
	}

	// Dummy methods so that modules do not need to implement all hooks
	bool ModuleBase::configureHook()
	{
		return true;
	}
	bool ModuleBase::startHook()
	{
		return true;
	}
	void ModuleBase::stopHook()
	{
	}
	void ModuleBase::cleanupHook()
	{
	}

	bool ModuleBase::configure()
	{
		return configureHook();
	}
	bool ModuleBase::start()
	{
		return startHook();
	}
	void ModuleBase::update()
	{
		updateHook();
	}
	void ModuleBase::stop()
	{
		stopHook();
	}
	void ModuleBase::cleanup()
	{
		cleanupHook();
	}

	void ModuleBase::setDescription(std::string desc)
	{
		this->desc = desc;
	}
	std::string ModuleBase::getDescription() const
	{
		return desc;
	}
	std::string ModuleBase::getPropertyValue(PropertyBase* prop) const
	{
		return prop->toString();
	}
	void ModuleBase::setPropertyValue(PropertyBase* prop, std::string value)
	{
		prop->fromString(value);
	}
	std::string ModuleBase::getPortDescription(Port* port) const
	{
		return port->getDescription();
	}
	std::string ModuleBase::getPropertyDescription(PropertyBase* prop) const
	{
		return prop->getDescription();
	}
	std::vector<Port*> ModuleBase::getPorts()
	{
		return ports()->getPorts();
	}
	std::vector<PropertyBase*> ModuleBase::getProperties()
	{
		return properties()->getProperties();
	}

}
