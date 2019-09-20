/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef MODULEBASE_HPP_
#define MODULEBASE_HPP_

#include <string>
#include <map>
#include <vector>
#include <list>
#include <sstream>

#include "NetFwd.hpp"
#include "TypeKit.hpp"

/**
 * \brief Namespace for all basic RCC classes
 *
 * Contents of the RPI namespace are generally compiled into the RealtimeRCC binary and the
 * librcc.so shared library
 */
namespace RPI
{

	//class Net;
	//class Module;
	//class DebugModule;

	/**
	 * \brief Base class for Properties
	 *
	 * Properties are part of a module, storing a constant value which can be set upon creation.
	 * Properties need support for converting native data type to std::string and vice versa. Instances
	 * of class Typekit are used for conversion.
	 */
	class RTT_EXPORT PropertyBase
	{
	private:
		std::string name;
		std::string desc;
	protected:
		TypeKit* typekit;
	public:
		/**
		 * \brief Get type of property (as string)
		 */
		virtual TypeKit* getType() const = 0;

		/**
		 * Creates a new property object
		 * \param name Name of the property (used in communication, no special chars, no whitespace etc.)
		 * \param desc Description of property (only used for human machine interface, all usual chars allowed)
		 */
		PropertyBase(const std::string& name, const std::string& desc);

		virtual ~PropertyBase();

		/**
		 * \brief get name of property
		 * \return name of propert
		 */
		std::string getName() const;
		/**
		 * \brief Update human-readable description of property
		 * \param name Human-readable description of property
		 */
		void setDescription(const std::string& desc);
		/**
		 * \brief Get human-readable description of property
		 */
		std::string getDescription() const;
		/**
		 * \brief Write content of property to string
		 */
		virtual std::string toString() const = 0;
		/**
		 * \brief Update content of property from string value
		 * \param value New value for property. Typekit tries to convert string to proper type.
		 */
		virtual void fromString(const std::string& value) = 0;
	};

	/**
	 * \brief Template implementation for properties
	 */
	template<class T> class RTT_EXPORT Property: public PropertyBase
	{
		T value;
	public:
		Property(const std::string& name, const std::string& desc) :
				PropertyBase(name, desc), value()
		{
			typekit = TypeKits::getInstance()->getTypeKit(typeid(T));
		}
		Property(const std::string& name, const std::string& desc, const T& value) :
				PropertyBase(name, desc), value(value)
		{
			typekit = TypeKits::getInstance()->getTypeKit(typeid(T));
		}
		virtual ~Property()
		{
		}
		/**
		 * \brief Update property
		 */
		void set(const T& value)
		{
			this->value = value;
		}
		/**
		 * \brief Retrieve value from property
		 */
		const T& get() const
		{
			return this->value;
		}
		std::string toString() const
		{
			std::stringstream result;
			typekit->toString(&this->value, result);
			return result.str();
		}

		void fromString(const std::string& value)
		{
			std::stringstream input;
			input << value;
			typekit->fromString(&this->value, input);
		}
		TypeKit* getType() const
		{
			return typekit;
		}
	};

	/**
	 * \brief Input or Output port
	 *
	 * Modules use ports to communication. An input port can be connected to exactly one output port,
	 * but an output port can be connected with an arbitrary number of input ports
	 */
	class RTT_EXPORT Port
	{
	private:
		std::string name;
		std::string desc;
		Module* inModule;
	protected:
		TypeKit* typekit;
		/**
		 * \brief Current net cycle counter
		 */
		long getNetCurrentCycle() const;

		/**
		 * \brief Last net cycle port has been updated
		 */
		virtual long getLastWriteCycle() const = 0;
	public:
		/**
		 * \brief Constructor for creating a new port
		 *
		 * IMPORTANT: Constructor MUST NOT dereference module, will most likely not being already constructed
		 *
		 * \param name Name of the port, will be used in communication, no special chars, no whitespace, etc
		 * \param module Pointer to module the port is used in. Module object will not be created, so dereferencing
		 *   pointer is invalid
		 */
		Port(const std::string& name, Module* module);

		virtual ~Port();

		/**
		 * \brief Pointer to value of port
		 */
		virtual const void* getValuePtr() const = 0;
		/**
		 * \brief Last write cycle pointer
		 *
		 * Pointer to long variable containing the last net cycle this port has been written
		 */
		virtual const long* getLastWriteCyclePtr() const = 0;
		/**
		 * \brief Name of port
		 */
		std::string getName() const;
		/**
		 * \brief Human-readable description of port
		 */
		std::string getDescription() const;
		/**
		 * \brief Sets human-readable description of port
		 * \param desc Description
		 */
		void setDescription(const std::string &desc);
		/**
		 * \brief Checks port for null value
		 * \return false, if port has been written in current net cycle
		 *
		 * If a port has not been written to in the current net cycle, the data
		 * is considered null.
		 */
		bool isNull() const;
		/**
		 * \brief Type of port
		 */
		virtual TypeKit* getType() const = 0;
		/**
		 * \brief Pointer to module the port belongs to
		 */
		Module* getModule() const;

		virtual bool connectWith(Port* port) = 0;
		/**
		 * \brief Port is connected
		 */
		virtual bool connected() const = 0;
		/**
		 * \brief Alias for connected()
		 */
		virtual bool ready();
		/**
		 * \brief Port direction
		 * \return true if port is output port, false if port is input port
		 */
		virtual bool isOutPort() const = 0;
	};

	/**
	 * \brief Generic input port implementation
	 */
	template<class T> class RTT_EXPORT InPort: public Port
	{
	private:
		const T* value;
		const long* lastWriteCycle;
	public:
		const void* getValuePtr() const
		{
			return value;
		}

		const long* getLastWriteCyclePtr() const
		{
			return lastWriteCycle;
		}

		long getLastWriteCycle() const
		{
			if(lastWriteCycle != NULL)
				return *lastWriteCycle;
			return 0;
		}

		TypeKit* getType() const
		{
			return typekit;
		}


		// Constructor MUST NOT dereference module, will most likely not being already constructed
		InPort(const std::string& name, Module* module) :
			Port(name, module), value(NULL), lastWriteCycle(NULL)
		{
			typekit = TypeKits::getInstance()->getTypeKit(typeid(T));
		}

		virtual ~InPort()
		{
		}

		bool connected() const
		{
			return value != NULL;
		}

		/**
		 * \brief Read input port with configurable value for unconnected port
		 * \param defaultvalue Value to return if port is unconnected
		 * \return value of port. If port value is null, last value is returned (NOT defaultvalue)
		 *
		 * If the port is not connected, the specified default value will be used. If port is null,
		 * no special action is taken and last value will be returned. The caller must use isNull()
		 * to detect old data.
		 */
		const T Get(T defaultvalue) const
		{
			if(value == NULL)
				return defaultvalue;
			else
				return *value;
		}

		/**
		 * \brief Read input port with default value for unconnected port
		 * \return value of port. If port value is null, last value is returned. If port is unconnected,
		 * 	default value of port type is returned
		 *
		 * If the port is not connected, the default value of the port's type will be used. If port is null,
		 * no special action is taken and last value will be returned. The caller must use isNull()
		 * to detect old data.
		 */
		const T Get() const
		{
			if (value == NULL)
				return T();
			else
				return *value;
		}
		/**
		 * \brief Read input port with default value for unconnected port
		 * \return value of port. If port value is null, last value is returned. If port is unconnected,
		 * 	default value is read from given property
		 * \param defaultvalue Property to read default value from
		 *
		 * If the port is not connected, the default value will be read from given property. If port is null,
		 * no special action is taken and last value will be returned. The caller must use isNull()
		 * to detect old data.
		 */
		const T Get(Property<T> defaultvalue) const
		{
			if(value == NULL)
				return defaultvalue.get();
			else
				return *value;
		}

		/**
		 * \brief Read input port only if value is not null
		 * \return Current value of input port
		 * \param defaultvalue Value to return in case port is not connected, or port value is null
		 *
		 * In contrast to the Get(...) methods, this method never reports old data. If port is
		 * unconnected, or no current data is available, given default value is returned
		 */
		const T GetNoNull(T defaultvalue) const
		{
			if(this->isNull())
				return defaultvalue;
			else
				return Get(defaultvalue);
		}

		/**
		 * \brief Read input port only if value is not null
		 * \return Current value of input port
		 * \param defaultvalue Value to return in case port is not connected, or port value is null
		 *
		 * In contrast to the Get(...) methods, this method never reports old data. If port is
		 * unconnected, or no current data is available, given default value is returned
		 */
		const T GetNoNull(Property<T> defaultvalue) const
		{
			if(this->isNull())
				return defaultvalue.get();
			else
				return Get(defaultvalue);
		}

		bool connectWith(Port* port)
		{
			if (port->getType() != getType())
				return false;
			this->value = (T*) port->getValuePtr();
			this->lastWriteCycle = port->getLastWriteCyclePtr();
			return true;
		}

		bool isOutPort() const
		{
			return false;
		}

	};

	/**
	 * \brief Generic output port implementation
	 */
	template<class T> class RTT_EXPORT OutPort: public Port
	{
	private:
		T value;
		long lastWriteCycle;
	public:
		const void* getValuePtr() const
		{
			return &value;
		}

		virtual ~OutPort()
		{

		}

		const long* getLastWriteCyclePtr() const
		{
			return &lastWriteCycle;
		}

		long getLastWriteCycle() const
		{
			return lastWriteCycle;
		}

		TypeKit* getType() const
		{
			return typekit;
		}

		// Constructor MUST NOT dereference module, will most likely not being already constructed
		OutPort(const std::string& name, Module* module) :
			Port(name, module), value(), lastWriteCycle(-1)
		{
			typekit = TypeKits::getInstance()->getTypeKit(typeid(T));
		}

		// Constructor MUST NOT dereference module, will most likely not being already constructed
		OutPort(const std::string& name, Module* module, const T& value) :
			Port(name, module), lastWriteCycle(-1)
		{
			typekit = TypeKits::getInstance()->getTypeKit(typeid(T));
			this->value = value;
		}

		bool connected() const
		{
			return false;
		}

		bool connectWith(Port*)
		{
			return false;
		}

		/**
		 * \brief Set output port
		 * \param value New port value
		 */
		void Set(const T& value)
		{
			this->value = value;
			this->lastWriteCycle = getNetCurrentCycle();
		}

		bool isOutPort() const
		{
			return true;
		}

		/**
		 * \brief Set output port to null
		 *
		 * Calling this function sets the port to null state. It does not erase any data previously
		 * written to the port, but does only tell all connected ports the data is outdated.
		 */
		void SetNull()
		{
			this->lastWriteCycle = -1;
		}
	};

	/**
	 * \brief Provides access to all properties of a module
	 */
	class RTT_EXPORT PropertyBag
	{
	private:
		std::vector<PropertyBase*> props;
	public:
		typedef std::vector<PropertyBase*> Properties;
		/**
		 * \brief Add property to module
		 * \param prop Property
		 */
		void addProperty(PropertyBase* prop);
		/**
		 * \brief Add property with human-readable description to module
		 * \param prop Property
		 * \param desc Description
		 */
		void addProperty(PropertyBase* prop, std::string desc);
		/**
		 * \brief Retrieves list of all properties of module
		 */
		Properties getProperties() const;
		/**
		 * \brief Searches for named property
		 * \param name Name of property
		 */
		PropertyBase* find(std::string name) const;
	};

	/**
	 * \brief Provides access to all ports of a module
	 */
	class RTT_EXPORT PortInterface
	{
	private:
		std::multimap<std::string, Port*> ports;
	public:
		/**
		 * \brief Add port with description to module
		 * \param port Port to add
		 * \param desc Human-readable description
		 *
		 * The given port is added to the module's ports. The name of the port
		 * is extracted by dereferencing port and calling getName()
		 */
		void addPort(Port* port, std::string desc);
		/**
		 * \brief Add port to module
		 * \param port Port to add
		 *
		 * The given port is added to the module's ports. The name of the port
		 * is extracted by dereferencing port and calling getName()
		 */
		void addPort(Port* port);
		/**
		 * \brief Add named port to module
		 * \param name Name of the port
		 *
		 * The given port is added to the module's ports. The name of the port itself
		 * is ignored, the supplied name is used instead
		 */
		void addNamedPort(std::string name, Port* port);
		/**
		 * \brief Retrieves all ports for given name
		 * \param name Name of port
		 * \return List of all ports with given name
		 */
		std::list<Port*> getPort(const std::string& name) const;
		/**
		 * \brief Retrieves a single port for a given name
		 * \param name Name of port
		 * \return Pointer to port with requested name. If no port, or more than one port
		 *   with the given name exists, NULL is returned
		 */
		Port* getSinglePort(const std::string& name) const;
		/**
		 * \brief Retrieves the first port or a given name
		 * \param name Name of port
		 * \return Pointer to the first port with requested name. NULL, if no port exists with given name
		 *
		 * getFirstPort always returns one port with the given name. No guarantees on order of ports is given.
		 */
		Port* getFirstPort(const std::string& name) const;
		std::vector<Port*> getPorts() const;
		std::vector<std::string> getPortNames() const;
	};

	/**
	 * \brief Base class for all RPI modules
	 */
	class RTT_EXPORT ModuleBase
	{
	public:
		ModuleBase(const std::string& name, Net* net);
		virtual ~ModuleBase();

		/**
		 * \brief Human-readable name of module
		 */
		std::string getName() const;
		/**
		 * \brief Interface to module's ports
		 */
		PortInterface* ports();
		/**
		 * \brief Interface to module's properties
		 */
		PropertyBag* properties();

		/**
		 * \brief Hook called during configuration of module
		 *
		 * The configureHook() is called during the creation of an RPI net. If the configuration
		 * of the module is wrong, the module must return false to indicate failure. If a single
		 * module fails, the whole net will be rejected.
		 */
		virtual bool configureHook();
		/**
		 * \brief Hook called on start of realtime execution of module
		 *
		 * The startHook() will be called directly before entering realtime execution of RPI net.
		 * If a single module returns false, the start of the net will be aborted.
		 */
		virtual bool startHook();
		/**
		 * \brief Hook called in every RPI net execution cycle
		 *
		 * The updateHook() is executed in every cycle of the RPI net execution. The execution is
		 * performed in real-time context, therefore no memory allocation etc. must be performed.
		 */
		virtual void updateHook() = 0;
		/**
		 * \brief Hook called after module execution has been finished
		 */
		virtual void stopHook();
		/**
		 * \brief Hook called before destruction of module
		 *
		 * The cleanupHook() should be used to release ressources aquired during configureHook().
		 * Alternatively, the destructor can be used.
		 */
		virtual void cleanupHook();

		bool configure();
		bool start();
		void update();
		void stop();
		void cleanup();

		void setDescription(std::string desc);
		std::vector<Port*> getPorts();
		std::vector<PropertyBase*> getProperties();
		std::string getDescription() const;
		std::string getPortDescription(Port* port) const;
		std::string getPropertyDescription(PropertyBase* prop) const;
		std::string getPropertyValue(PropertyBase* prop) const;
		void setPropertyValue(PropertyBase* prop, std::string value);

	protected:
		PortInterface portInterface;
		PropertyBag propInterface;
		std::string name;
		std::string desc;
	};

}
#endif /* MODULEBASE_HPP_ */
