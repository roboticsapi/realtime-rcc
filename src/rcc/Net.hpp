/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef RPINET_HPP_
#define RPINET_HPP_

#include "NetFwd.hpp"

#include <string>
#include <map>
#include <set>
#include <list>
#include <memory>
#include <vector>

#include <rtt/os/Mutex.hpp>

#include "ModuleFwd.hpp"
// unfortunately this is necessary due to VC++ compiler
#include "DeviceInstanceT.hpp"
#include "NetModulesFwd.hpp"
#include "NetExecutorFwd.hpp"
#include "NetAST.hpp"
#include "FragmentFwd.hpp"
#include "DeviceFwd.hpp"
#include "RoundRobinLog.hpp"

namespace RPI
{
	std::string inline HTTPServer_state_text(NetState status)
	{
		switch (status)
		{
		case Loading:
			return "LOADING";
		case Rejected:
			return "REJECTED";
		case Ready:
			return "READY";
		case Running:
			return "RUNNING";
		case Terminated:
			return "TERMINATED";
		case Cancelling:
			return "CANCELLING";
		case Scheduled:
			return "SCHEDULED";
		case Unloading:
			return "TERMINATED";
		default:
			return "INVALID";
		}

	}

	class NetErrorState
	{
	public:
		NetErrorState()
		{
			errorMajor = NoError;
			errorMinor = 0;
			errorMessage = "";
			errorBlock = "";
		}

		NetErrorMajor errorMajor;
		int errorMinor;
		std::string errorMessage;
		std::string errorBlock;
	};

	/**
	 * This class implements an RPI net
	 */
	class RTT_EXPORT Net
	{
	public:
		/**
		 * Creates a new task context for RPI net interpreter
		 * \param name unique identifier for this net
		 * \param factory a ModuleFactoryMap providing all necessary modules (use RPIModuleLoader)
		 * \param session the session this net belongs to
		 */
		Net(const std::string& name, const std::string& desc, ModuleFactoryMap* factory, Session_ID_t session, const rpi_fragment& netfragment, double frequency, bool isRealtime);
		~Net();

		// Runtime functions
		bool configure();
		// frequency and isrealtime will be silently ignored if net is scheduled
		bool start();
		void update();
		void stop();
		void cleanup();

		NetState getNetState() const;
		void abort();
		void cancel();
		void unload();
		bool getUnloadFlag() const;

		/**
		 * schedule the execution of this net as soon as all nets which use required
		 * resources have release them
		 * Returns false if another net is already scheduled which need same resource
		 */
		bool schedule(Net* takeovernet);

		void setNetExecutor(NetExecutor* executor);

		// Name, data
		std::string getName() const;
		std::string getDescription();
		std::string getNetXML();
		std::string getNetDIO();

		// Netcomm funcitonality
		void addCommunicationInProperty(const std::string& name, Netcomm_In* property);
		void removeCommunicationInProperty(const std::string& name);
		std::map<std::string, std::list<Netcomm_In*> > getCommunicationInProperties() const;
		void addCommunicationOutProperty(const std::string& name, Netcomm_Out* property);
		void removeCommunicationOutProperty(const std::string& name);
		std::map<std::string, Netcomm_Out*> getCommunicationOutProperties() const;

		std::map<std::string, std::string> readCommunicationInProperties() const;
		void writeCommunicationInProperties(const std::map<std::string, std::string>& values);
		Netcomm_Map_t readCommunicationOutProperties() const;
		std::shared_ptr<NetcommData> getCommunicationData(const std::string& key) const;

		void addCancelModule(Cancel*);
		void addTakeoverModule(Takeover*);
		void addDebugModule(DebugModule*);

		// Debug functionality
		std::vector<DebugModule*> getDebugModules() const;
		std::vector<Module*> getModules() const;

		// Session ID
		Session_ID_t getSessionId() const;

		// Error
		std::list<NetErrorState> netErrorState;

		// Mutex for NetComms
		std::shared_ptr<RTT::os::Mutex> netCommMutex;

		// Timing
		RTT::os::TimeService::nsecs maxTime, minTime, maxCycleTime, minCycleTime, avgCycleTimeSum;
		long getCurrentCycle() const;

		RTT::os::TimeService::nsecs getTime() const;

		void configurationError(NetConfigurationError error, const std::string& message, const std::string& block = "");

		// cycle information from NetExecutor
		double getNetFrequency() const;
		bool isRealtimeNet() const;

		void triggerCrashDump();
		void processCrashDump();
	private:
		// Runtime functions
		void ParentAborted();

		bool acquireResources();
		std::set<std::string> getRequiredResources() const;

		// internal data

		// The root fragment
		Fragment* rootFragment;
		// map of the factories to create the modules
		ModuleFactoryMap* factoryMap;

		// Helper lists for special modules
		std::vector<DeviceInstanceT<Device> > usedDevices;
		std::vector<Module*> preModules;
		//std::vector<Module*> sensorModules;
		//std::vector<Module*> actuatorModules;

		// array to manage boolean values for resource usage
		bool* resourceUsagePtr;
		int resourceUsageCount;

		Net* scheduledNet;
		Net* scheduledByNet;

		NetState netState;

		std::list<Cancel*> cancelModules;
		std::list<Takeover*> takeoverModules;
		Termination* terminationModule;

		std::map<std::string, std::list<Netcomm_In*> > communicationInProperties;
		std::map<std::string, Netcomm_Out*> communicationOutProperties;
		std::map<std::string, std::shared_ptr<NetcommData> > communicationData;

		std::vector<DebugModule*> debug_modules;

		bool unloadFlag, currentlyRunning;
		bool noScheduledNetStart;

		Session_ID_t session;

		RTT::os::TimeService::nsecs last;
		int timeoverrun;
		long currentCycle;

		rpi_fragment netfragment;
		// XML representation - remove!
		//std::string netxml;
		std::string name;
		std::string desc;

		NetExecutor* netExecutor;

		double netFrequency;
		bool isRealtime;
		bool crashDumpRequested;

		RoundRobinLog<RTT::os::TimeService::nsecs> *executionTimeDumper, *actuatorTimeDumper, *totalTimeDumper;
		RoundRobinLog<long>* executionCycleDumper;
	};

}

#endif /* RPINET_HPP_ */
