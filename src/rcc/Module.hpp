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

// Remove when parseString method has been refactored
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>


namespace RPI
{
	/**
	 * \brief Management of resources used in net
	 *
	 * Some resources like robot joints can only be controlled by one net at any time.
	 * All other nets requesting these resources must be rejected, except they are scheduled
	 * directly after another net which has a superset of acquired resources.
	 */
	class ResourceManager
	{
	public:
		/**
		 * \brief Singleton access to ResourceManager
		 */
		static ResourceManager* getResourceManager();

		/**
		 * \brief Get users for given resource
		 */
		std::string getResource(const std::string& resourceID) const;
		/**
		 * \brief Add usage of resource
		 */
		void addResource(const std::string& resourceID, const std::string& netID);
		/**
		 * \brief Remove usage of resource
		 */
		void removeResource(const std::string& resourceID);
		/**
		 * \brief Remove usages on all resources a net currently holds
		 */
		void removeNetUsage(const std::string& netID);

		/**
		 * \brief Mutex to obtain before performing any resource related operation
		 */
		RTT::os::Mutex resourceMutex;
		/**
		 * \brief Mutex to obtain for scheduling of nets
		 */
		RTT::os::Mutex scheduleNetMutex;
	private:
		ResourceManager();
		std::map<std::string, std::string> resourceMap;

		static ResourceManager* theManager;
	};

	/**
	 * \brief Base class for all RPI modules
	 */
	class RTT_EXPORT Module: public ModuleBase
	{
	public:
	protected:
		/**
		 * \brief the RPI net containing the module
		 */
		Net* inNet;
	public:
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

		Module(const std::string& name, Net* net);

		virtual ~Module();

	protected:
		/**
		 * Returns the list of required resources for this module. If a module requires
		 * a resource, a net can only be started if no other net requires this resource
		 */
		virtual std::set<std::string> getResourceNames() const;
	public:
		virtual const std::set<std::string>& getCachedResourceNames() const;

		virtual bool checkResources() const;

		/**
		 * Registers the module at the resource handler for the current net
		 * aquireResource must be called under the same Mutex as checkResource to avoid Resources been taken
		 * by other threads after checking
		 */
		virtual void acquireResources();

		/**
		 * Unregisters the module at the resource handler
		 */
		virtual void releaseResources();

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
			std::vector<T> result;
			typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
			boost::char_separator<char> sep("{},");
			tokenizer tokens(data, sep);
			for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter)
			{
				try
				{
					result.push_back(boost::lexical_cast<T>(*tok_iter));
				} catch (boost::bad_lexical_cast &)
				{
					result.push_back(T());
				}
			}
			return result;
		}
	private:
		mutable std::set<std::string> resourceNamesCache;
		mutable bool resourceNamesCacheOk;
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
