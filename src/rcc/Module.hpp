/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef RPIMODULE_HPP_
#define RPIMODULE_HPP_
#include <string>
#include <vector>
#include <set>
#include "ModuleBase.hpp"
#include "DeviceInstanceFwd.hpp"
#include "NetFwd.hpp"

#include <rtt/os/Mutex.hpp>
#include <rtt/os/TimeService.hpp>

#include "TypeKit.hpp"


namespace RPI
{


	/**
	 * \brief Base class for all RPI modules
	 */
	class RTT_EXPORT Module: public ModuleBase
	{
	public:
		Module(const std::string& name, Net* net);

		virtual ~Module();

		/**
		 * \brief array which contains pointers to boolean values for resource usage
		 */
		bool** resourceUsed;
		/**
		 * \brief number of resources, size of array resourceUsed
		 */
		int resourceCount;

		RTT::os::TimeService::nsecs minExecutionTime, maxExecutionTime, totalExecutionTime;

		/**
		 * allocate the resource array, put into module to avoid potential problems with different
		 * implementations of new / delete[] in Net.cxx and loadable modules
		 */
		void allocateResourceArray(int size);

		/**
		 * \brief Pointer to Net containing the module
		 */
		const Net* getInNet() const;

		/**
		 * Returns the list of required resources for this module. If a module requires
		 * a resource, a net can only be started if no other net requires this resource
		 */
		virtual std::set<std::string> getResourceNames() const;

		/**
		 * Returns current cycle count of net containing this module
		 */
		long getNetCurrentCycle() const;

		/**
		 * Returns whether module is a special "Pre" module, which needs special attention
		 * during net loading phase
		 */
		virtual bool isPre()
		{
			return false;
		}

		/**
		 * Returns whether module is a sensor, for sensors the updateSensor() method will
		 * be called prior net execution
		 */
		virtual bool isSensor()
		{
			return false;
		}

		/**
		 * Returns whether module is an actuator. Actuators will have the updateActuator() method
		 * called after net execution
		 */
		virtual bool isActuator()
		{
			return false;
		}

		/**
		 * Update hook for Pre-Modules
		 */
		virtual void updatePre()
		{
		}

		/**
		 * Method will be called for
		 */
		virtual void updateSensor()
		{
		}

		virtual void updateActuator()
		{
		}

		virtual std::string getDeviceName()
		{
			return "";
		}

		virtual std::vector<Module*> getLeafModules();


		bool portConnected(Port* port);
		bool deviceAvailable(DeviceInstance* devins, std::string deviceid);

		/**
		 * Tokenize std::string, convert parts into proper datatype and store in vector
		 * Example: parsestd::string<int>("{0,1,2}") will return vector<int> with items
		 * 0, 1 and 2
		 */
		template<class T>
		static inline std::vector<T> parseString(std::string const& data)
		{
			RPI::Array<T> ret;
			RPI::TypeKit* typekit = RPI::TypeKits::getInstance()->getTypeKit<Array<T>>();
			typekit->fromString(&ret, data);

			std::vector<T> result;
			for(int i=0; i< ret.getSize(); i++) {
				result.push_back(ret[i]);
			}
			return result;
		}

	protected:
		/**
		 * \brief the RPI net containing the module
		 */
		Net* inNet;

	};

	class RTT_EXPORT ActiveModule: public Module
	{
	public:
		ActiveModule(const std::string& name, Net* net);

		virtual ~ActiveModule();
	protected:
		bool inline active() const
		{
			return inActive.Get(true);
		}

		InPort<bool> inActive;

	};


	class RTT_EXPORT StatefulModule: public ActiveModule
	{
	public:
		StatefulModule(const std::string& name, Net* net);

		virtual ~StatefulModule();
	protected:
		bool inline reset() const
		{
			return inReset.Get(false);
		}

		InPort<bool> inReset;

	};

	class RTT_EXPORT ModuleFactory
	{
	protected:
		ModuleFactory();

	public:
		virtual ~ModuleFactory();

		std::string creatorExtension;

		virtual Module* createInstance(const std::string& name, Net* net) = 0;
		virtual void destroyInstance(Module*) = 0;
	};

	template<class C> class ModuleFactoryT: public ModuleFactory
	{
	public:
		ModuleFactoryT():ModuleFactory()
		{

		}

		Module* createInstance(const std::string& name, Net* net)
		{
			return new C(name, net);
		}
		void destroyInstance(Module* module)
		{
			delete module;
		}

	};

}
#endif /* RPIMODULE_HPP_ */
