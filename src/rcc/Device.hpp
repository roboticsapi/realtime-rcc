/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DRIVER_HPP_
#define DRIVER_HPP_

#include "DeviceFwd.hpp"

#include <set>
#include <map>
#include <string>
#include <vector>
#include <list>
#include <istream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <rtt/rtt-config.h>
#include "CrashDumperFwd.hpp"


namespace RPI
{
	class HTTPHandler;
	class Module;

	/**
	 * \brief Base class for devices (robots, ...)
	 *
	 * A device is a configurable entity that usually exclusively controls some pice
	 * of hardware. Devices can be loaded and unloaded at runtime. Parameters are used
	 * for configuration. RPI modules can query the Registry for a certain device to
	 * utilize.
	 */
	class RTT_EXPORT Device
	{
	public:
		/**
		 * Creates a new Device instance using given parameters
		 * \param initialparameters Initial parameters for device
		 */
		Device(std::string name, const parameter_t& initialparameters);
		virtual ~Device();

		/**
		 * \brief List of mutable parameters
		 *
		 * This function returns a list of parameters which can be changed at runtime.
		 * By default, no parameter is changeable once a device has been created.
		 */
		virtual std::set<std::string> getMutableParameters() const = 0;
		/**
		 * \brief Tells device that parameters have been changed
		 */
		virtual void updateParameters() = 0;
		/**
		 * \brief Set device parameter
		 * \param name Parameter name
		 * \param value Parameter value
		 */
		void setParameter(std::string const& name, std::string const& value);
		/**
		 * \brief Get device parameter
		 * \param name Parameter name
		 * \param defaultvalue Default value (empty string if not specified)
		 * \return Parameter value
		 */
		std::string getParameter(const std::string& name, const std::string& defaultvalue = "") const;

		template<class T>
		T getParameterT(std::string name, T defaultvalue) const
		{
			std::string lowername = boost::to_lower_copy(name);
			parameter_t::const_iterator it = parameters.find(lowername);
			try
			{
				if (it != parameters.end())
				{
					return boost::lexical_cast<T>(it->second);
				}
			} catch (boost::bad_lexical_cast&)
			{

			}
			return defaultvalue;

		}


		/**
		 * \brief Get list of all parameters
		 */
		std::set<std::string> getParameters() const;

		/**
		 * \brief Locks device before updating
		 *
		 * The lockDevice() function is called before RPI net enters sensor or actuator update
		 * routine. The device should perform all necessary locking to prevent concurrent
		 * access to critical resources. After sensor/actuator updates have been performed,
		 * unlockDevice() is called.
		 */
		virtual void lockDevice();

		/**
		 * \brief Unlocks device after updating
		 * \see lockDevice()
		 */
		virtual void unlockDevice();

		/**
		 * \brief check whether device is removable
		 *
		 * By default, every device can be removed during runtime (if it is not actively used
		 * by a running command).
		 */
		virtual bool isRemovable()
		{
			return true;
		}

		/**
		 * \brief updatesemergency stop state
		 * \param inEStop emergency stop requested
		 *
		 * This function is called with parameter inEStop true if an emergency stop is requested and
		 * with value false if emergency stop is not active anymore. This function might be called
		 * subsequently with the same parameter again.
		 */
		virtual void setEStop(bool inEStop) = 0;

		/**
		 * \brief get the device state
		 *
		 * Retrieves the state of the device
		 */
		virtual DeviceState getDeviceState() const = 0;

		std::list<std::string> implementedDeviceInterfaces;

		std::string getName() const;

		void triggerCrashDump();
		void processCrashDump();

	protected:
		std::list<RPI::CrashDumper*> crashDumpers;
		parameter_t parameters;
		std::string name;
		bool crashDumpRequested;

		void addCrashDumper(CrashDumper* crashDumper);
		static void merge_default_parameters(parameter_t& parameters, const parameter_t& defaultparamaeters);
	};

	/**
	 * \brief Information class for HTTP handlers
	 */
	class HTTPHandlerInfo
	{
	public:
		/**
		 * \param path path where handler should be registered
		 * \param handler Pointer to the handler to be registered
		 *
		 * The constructor crates a new handler information object, taking the desired
		 * path and pointer to the handler object
		 */
		HTTPHandlerInfo(const std::string& path, HTTPHandler* handler)
		{
			this->path = path;
			this->handler = handler;
		}
		std::string path;
		HTTPHandler* handler;
	};

	typedef std::vector<HTTPHandlerInfo> HTTPHandlerList;

	/**
	 * \brief Factory for device creation
	 *
	 * The concrete factory implementation is provided for every device by each extension module.
	 * Thereby it can be guaranteed, that extensions will be using appropriate new and delete
	 * operators, even if extension has been compiled using another compiler that the main executable.
	 *
	 * DeviceFactory is pure virtual, for each device there must be specific DeviceFactoryT<Device> type.
	 */
	class RTT_EXPORT DeviceFactory
	{
	public:
		/**
		 * \brief ID of the extension that registered this device factory
		 */
		std::string creatorExtension;

		/**
		 * \brief Create new instance of device
		 * \param name Name for the new device instance
		 * \param parameter Initial parameters for device creation
		 *
		 * The device factory creates a new instance of the device, sets the device up properly and
		 * returns a pointer to the device.
		 */
		virtual Device* createInstance(const std::string& name, const parameter_t& parameter) = 0;

		/**
		 * \brief Destroy existing instance of device
		 * \param name Name of the device to be destroyed
		 * \param device Instance of device
		 *
		 * The device factory destroys a given instance of a device. The device's name is required
		 * in order to be able to unregister all HTTP handlers.
		 */
		virtual bool destroyInstance(const std::string& name, Device* device) = 0;

		/**
		 * \brief Hook for HTTP handler cleaning
		 *
		 * The cleanupHTTPHandlers() method is a pure convenience method to facilitate
		 * the removal of HTTP handlers. Because the removal of HTTP handlers is almost
		 * identical for every device, a generic template is provied in DeviceFactoryT.
		 */
		virtual void cleanupHTTPHandlers() = 0;

		virtual ~DeviceFactory();
	protected:
		/**
		 * \brief Get List of HTTP handlers
		 */
		const HTTPHandlerList& getHTTPHandlers() const;

		/**
		 * \brief Protected constructor of DeviceFactory
		 *
		 * Constructor is protected, so no instance of DeviceFactory can be created. Use concrete
		 * variant of DeviceFactoryT<T> instead.
		 */
		DeviceFactory(const HTTPHandlerList& httpHandlers);
	private:
		/**
		 * \brief List of hTTP handlers
		 */
		HTTPHandlerList httpHandlers;
	};

}
#endif /* DRIVER_HPP_ */
