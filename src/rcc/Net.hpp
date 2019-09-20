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

#include "SynchronizationRuleFwd.hpp"
#include "scheduling/SchedulingConditionFwd.hpp"

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
		case Unloading:
			return "TERMINATED";
		case Scheduled:
			return "SCHEDULED";
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
		Net(const std::string& name, const std::string& desc, ModuleFactoryMap* factory, Session_ID_t session,
				const rpi_fragment& netfragment, double frequency, bool isRealtime);
		virtual ~Net();

		// Runtime functions
		bool configure();

		/**
		 * Run cycle, triggered by NetExecutor
		 *
		 * @return true, if update was performed completely, false if update was aborted due to scheduling
		 */
		bool update();

		void cleanup();
		void abort();
		void cancel();
		void unload();

		NetState getNetState() const;
		void setNetState(NetState);
		bool isRequiredState(int n, ...) const;
		bool isNotState(int n, ...) const;

		bool getUnloadFlag() const;

		const std::set<std::string>& getResourceNames() const;
		void releaseResources();
		void acquireResources();
		// notify net of resource loss
		void loseResources();

		void setNetExecutor(NetExecutor* executor);
		NetExecutor* getNetExecutor() const;

		// Name, data
		std::string getName() const;
		std::string getDescription();
		std::string getNetXML();
		std::string getNetDIO();

		// Netcomm funcitonality
		void addCommunicationInProperty(const std::string& name, Netcomm_In* property);
		void removeCommunicationInProperty(const std::string& name);
		std::map<std::string, std::list<Netcomm_In*> > getCommunicationInProperties() const;
		void addCommunicationOutProperty(const std::string& name, Netcomm_Out* property, bool report);
		void removeCommunicationOutProperty(const std::string& name);
		std::map<std::string, Netcomm_Out*> getCommunicationOutProperties() const;

		std::map<std::string, std::string> readCommunicationInProperties() const;
		void writeCommunicationInProperties(const std::map<std::string, std::string>& values);
		Netcomm_Map_t readCommunicationOutProperties() const;
		std::shared_ptr<NetcommData> getCommunicationData(const std::string& key) const;

		// Special Net modules
		void addCancelModule(Cancel*);

		void addCondition(std::shared_ptr<SynchronizationRule> cond);
		std::shared_ptr<SchedCond::SchedValue> getSchedValue(const std::string& ncname) const;
		std::list<std::shared_ptr<SynchronizationRule>> getConditions() const;

		// Debug functionality
		void addDebugModule(DebugModule*);
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

		std::string getNetDebug();
		std::string getNetDebug(int& startCycle);

		virtual std::map<std::string, RTT::os::TimeService::nsecs> getNetBlockMinTime();
		virtual std::map<std::string, RTT::os::TimeService::nsecs> getNetBlockMaxTime();
		virtual std::map<std::string, RTT::os::TimeService::nsecs> getNetBlockTotalTime();

		void triggerCrashDump();
		void processCrashDump();
	private:
		// Runtime functions

		void stop();

		// The root fragment
		Fragment* rootFragment;
		// map of the factories to create the modules
		ModuleFactoryMap* factoryMap;

		// Helper lists for special modules
		std::vector<DeviceInstanceT<Device> > usedDevices;
		std::vector<Module*> preModules;

		// array to manage boolean values for resource usage
		bool* resourceUsagePtr;
		int resourceUsageCount;

		// Name of all resources we need
		std::set<std::string> resourceNames;

		NetState netState;

		// special modules
		std::list<Cancel*> cancelModules;
		std::vector<DebugModule*> debug_modules;

		// pointer to outTerminate value
		bool* terminationState;

		std::map<std::string, std::list<Netcomm_In*> > communicationInProperties;
		std::map<std::string, Netcomm_Out*> communicationOutProperties;
		std::map<std::string, std::shared_ptr<NetcommData> > communicationData;

		std::map<std::string, std::pair<std::shared_ptr<TNetcommData<bool>>, std::shared_ptr<SchedCond::SchedValue>>> schedValues;

		std::list<std::shared_ptr<SynchronizationRule>> conditions;
		bool conditionsChanged;

		// Marker flags for execution
		bool unloadFlag, currentlyRunning;

		Session_ID_t session;

		// Time
		RTT::os::TimeService::nsecs last;
		int timeoverrun;
		long currentCycle;

		// Main fragment
		rpi_fragment netfragment;

		std::string name;
		std::string desc;

		NetExecutor* netExecutor;

		double netFrequency;
		bool isRealtime;
		bool stopExecution;

		// Prevent net from unloading
		mutable RTT::os::Mutex netUnloadMutex;
		bool crashDumpRequested;


	// Static methods for net creation etc.
	public:
		static bool scheduleCondition(std::shared_ptr<SynchronizationRule> cond);
		static bool scheduleNets(Sync_ID_t id, std::unique_ptr<SchedCond::SchedCondition> condition, std::set<Net_ID_t>, std::set<Net_ID_t>, std::set<Net_ID_t>, const std::string& description);

	private:
		static bool checkResourcesForExecutor(const std::set<std::string>& providedResourceNames, const std::set<std::string>& necessaryResources);



		RoundRobinLog<RTT::os::TimeService::nsecs> *executionTimeDumper, *actuatorTimeDumper, *totalTimeDumper, *rpiTimeDumper;
		RoundRobinLog<long>* executionCycleDumper;
	};

}

#endif /* RPINET_HPP_ */
