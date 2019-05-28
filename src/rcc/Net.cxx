/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <time.h>

#include "Net.hpp"
#include "NetParser.hpp"
#include "NetAST.hpp"
#include "NetModules.hpp"
#include "Registry.hpp"
#include "NetExecutor.hpp"
#include <rtt/Activity.hpp>
#include <queue>
#include "Fragment.hpp"
#include "CrashDump.hpp"
#include "DeviceInstanceT.hpp"
#include "RoundRobinLog.hpp"

namespace RPI
{
	using namespace std;
	using namespace RTT;
	using namespace RTT::os;

	Net::Net(const string& name, const string& desc, ModuleFactoryMap* factory, Session_ID_t session,
			const rpi_fragment& netfragment, double frequency, bool isRealtime) :
			maxTime(0), minTime(TimeService::InfiniteNSecs), maxCycleTime(0), minCycleTime(TimeService::InfiniteNSecs), preModules(), resourceUsagePtr(
					0), scheduledNet(0), scheduledByNet(0), session(session), last(0), timeoverrun(0), currentCycle(0), name(
					name), desc(desc), netFrequency(frequency), isRealtime(isRealtime), avgCycleTimeSum(0)
	{
		//this->netxml = netxml;
		this->netfragment = netfragment;

		factoryMap = factory;
		rootFragment = 0;
		terminationModule = 0;

		unloadFlag = false;
		currentlyRunning = false;
		noScheduledNetStart = false;
		crashDumpRequested = false;

		resourceUsagePtr = 0;
		resourceUsageCount = 0;

		netExecutor = 0;

		netCommMutex = make_shared<RTT::os::Mutex>();

		// Load the RPI net specified in "net"
		netState = Loading;

		// net is no longer TaskContext
		//this->setActivity(new RTT::PeriodicActivity(RTT::os::HighestPriority, RPI_CYCLE_TIME));

		executionTimeDumper = new RoundRobinLog<RTT::os::TimeService::nsecs>();
		executionTimeDumper->setName("executionTime");
		actuatorTimeDumper = new RoundRobinLog<RTT::os::TimeService::nsecs>();
		actuatorTimeDumper->setName("actuatorTime");
		totalTimeDumper = new RoundRobinLog<RTT::os::TimeService::nsecs>();
		totalTimeDumper->setName("totalTime");
		executionCycleDumper = new RoundRobinLog<long>();
		executionCycleDumper->setName("cycle");
	}

	Net::~Net()
	{
		// release any resources that may habe been held due to interleaving in stop() and unloading net
		// this operation should never be necessary, only as precaution
		RTT::os::MutexLock lock(ResourceManager::getResourceManager()->resourceMutex);

		ResourceManager::getResourceManager()->removeNetUsage(this->getName());

		communicationOutProperties.clear();
		communicationInProperties.clear();

		// Unload root fragment and termination primitive
		delete rootFragment;
		delete terminationModule;

		for (vector<DebugModule*>::iterator ti = debug_modules.begin(); ti != debug_modules.end(); ++ti)
		{
			delete (*ti);
		}
		debug_modules.clear();

		delete[] resourceUsagePtr;

		delete executionTimeDumper;
		delete actuatorTimeDumper;
		delete totalTimeDumper;
		delete executionCycleDumper;
	}

	void Net::update()
	{
		// "lock" unloading of net
		currentlyRunning = true;
		if (unloadFlag)
		{
			currentlyRunning = false;
			return;
		}

		if (last > 0)
		{
			TimeService::nsecs cycleTime = RTT::os::TimeService::Instance()->getNSecs(last);
			if (cycleTime > maxCycleTime)
				maxCycleTime = cycleTime;
			if (cycleTime < minCycleTime)
				minCycleTime = cycleTime;

			avgCycleTimeSum += cycleTime;
		}

		last = RTT::os::TimeService::Instance()->getNSecs();

		// net to be overtaken?
		if (scheduledNet)
			for (std::list<Takeover*>::iterator it = takeoverModules.begin(); it != takeoverModules.end(); ++it)
				(*it)->setTakeover(true);

		for (std::list<Cancel*>::iterator it = cancelModules.begin(); it != cancelModules.end(); ++it)
			if (netState == RPI::Running && (*it)->getCancel())
				netState = RPI::Cancelling;

		// reset resource usage
		for (int i = 0; i < resourceUsageCount; i++)
		{
			resourceUsagePtr[i] = false;
		}

		// read sensor values
		for (vector<DeviceInstanceT<Device> >::iterator it = usedDevices.begin(); it != usedDevices.end(); ++it)
			(*it)->lockDevice();

		rootFragment->updateSensor();

		for (vector<DeviceInstanceT<Device> >::iterator it = usedDevices.begin(); it != usedDevices.end(); ++it)
			(*it)->unlockDevice();

		// Update root fragment
		RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();
		rootFragment->updateHook();
		terminationModule->updateHook();

		TimeService::nsecs duration = RTT::os::TimeService::Instance()->getNSecs(start);
		rootFragment->totalExecutionTime += duration;
		if (duration > rootFragment->maxExecutionTime)
			rootFragment->maxExecutionTime = duration;
		if (duration < rootFragment->minExecutionTime)
			rootFragment->minExecutionTime = duration;

		currentCycle++;

		auto tAfterExec = RTT::os::TimeService::Instance()->getNSecs();
		executionTimeDumper->put(duration, tAfterExec);
		executionCycleDumper->put(currentCycle, tAfterExec);

		for (vector<Module*>::iterator ti = preModules.begin(); ti != preModules.end(); ++ti)
		{
			(*ti)->updatePre();
		}

		if (netExecutor != 0)
		{

			// write actuator values
			for (vector<DeviceInstanceT<Device> >::iterator it = usedDevices.begin(); it != usedDevices.end(); ++it)
				(*it)->lockDevice();

			rootFragment->updateActuator();

			for (vector<DeviceInstanceT<Device> >::iterator it = usedDevices.begin(); it != usedDevices.end(); ++it)
				(*it)->unlockDevice();

		}

		TimeService::nsecs cycleDuration = RTT::os::TimeService::Instance()->getNSecs(last);
		if (cycleDuration > maxTime)
			maxTime = cycleDuration;
		if (cycleDuration < minTime)
			minTime = cycleDuration;

		totalTimeDumper->put(cycleDuration);
		actuatorTimeDumper->put(RTT::os::TimeService::Instance()->getNSecs(tAfterExec));

#ifdef REALTIME
		if(duration > 0.9 * netFrequency * 10e9)
		timeoverrun++;

		if(timeoverrun > 5)
		{
			abort();
			RTT::log(RTT::Critical) << "Net execution took too long, execution stopped" << RTT::endlog();
		}
#endif

		if (terminationModule)
		{
			if (terminationModule->getFailState() != 0)
			{
				NetErrorState nes;
				nes.errorMajor = InternalFail;
				nes.errorMinor = terminationModule->getFailState();
				netErrorState.push_back(nes);
				abort();

			} else if (terminationModule->getTerminationState())
			{
				stop();
			}
		}
		currentlyRunning = false;
	}

	// external registry method
	bool Net::configure()
	{
		RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();
		taskmap_t taskmap;

		netState = Rejected;

		// abstract specification of the modules to load
		//std::list<rpi_module> blocks;

		Fragment* fragment = new Fragment(this, this->factoryMap, netfragment);

		rootFragment = fragment;

		if (!fragment->buildFragment())
		{
			return false;
		}

		if (fragment->ports()->getSinglePort("outTerminate"))
		{
			Termination* termination = new Termination("Termination", this);
			terminationModule = termination;
			termination->ports()->getSinglePort("inTerminate")->connectWith(
					fragment->ports()->getSinglePort("outTerminate"));
			if (fragment->ports()->getSinglePort("outFail"))
				termination->ports()->getSinglePort("inFail")->connectWith(fragment->ports()->getSinglePort("outFail"));
		} else
		{
			configurationError(NoTermination, "Missing outTerminate port on root fragment");
			return false;
		}

		if (!fragment->configure())
		{
			return false;
		}

		map<string, bool*> resourceUsage;

		// look up all resources used in this net and create pointertable
		std::vector<Module*> modules = fragment->getLeafModules();
		std::list<std::string> usedDeviceNames;

		for (vector<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			set<string> resources = (*it)->getCachedResourceNames();
			for (set<string>::iterator it2 = resources.begin(); it2 != resources.end(); ++it2)
			{
				resourceUsage[*it2] = 0;
			}

			if ((*it)->isPre())
			{
				preModules.push_back(*it);
			}

			if ((*it)->isSensor())
			{
				usedDeviceNames.push_back((*it)->getDeviceName());
				//sensorModules.push_back(*it);
			}
			if ((*it)->isActuator())
			{
				usedDeviceNames.push_back((*it)->getDeviceName());
				//actuatorModules.push_back(*it);
			}
		}

		// Devices must be sorted to avoid deadlocks
		usedDeviceNames.sort();
		usedDeviceNames.unique();

		for (list<string>::iterator it = usedDeviceNames.begin(); it != usedDeviceNames.end(); ++it)
		{
			DeviceInstanceT<Device> dev = DeviceInstanceT<Device>(*it);
			if (!dev.getDevice())
			{
				configurationError(DeviceNotFound, "Device " + (*it) + " not found");
				return false;
			}
			usedDevices.push_back(dev);
		}

		resourceUsageCount = resourceUsage.size();
		resourceUsagePtr = new bool[resourceUsageCount];

		bool* iresourceUsagePtr = resourceUsagePtr;

		for (vector<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			set<string> resources = (*it)->getCachedResourceNames();
			(*it)->allocateResourceArray(resources.size());

			//for (unsigned int i = 0; i < resources.size(); i++)
			int i = 0;
			for(const auto& res : resources)
			{
				//string instance = resources[i];
				if (!resourceUsage[res])
				{
					// allocate new boolptr
					resourceUsage[res] = iresourceUsagePtr++;
				}
				(*it)->resourceUsed[i] = resourceUsage[res];
				i++;
			}
		}

		RTT::log(RTT::Debug) << "create resources " << RTT::os::TimeService::Instance()->getNSecs(start) / 1e9
				<< RTT::endlog();
		start = RTT::os::TimeService::Instance()->getNSecs();

		netState = Ready;
		return true;
	}

	// external registry method
	bool Net::start()
	{
		{
			// Lock on resource management Mutex
			RTT::os::MutexLock lock(ResourceManager::getResourceManager()->resourceMutex);

			if (netState != Ready && netState != Scheduled)
				return false;

			if (!acquireResources())
				return false;
		}

		// No mutex must be held on calling start(), otherwise another thread might be blocked
		// in its update handler, effectively blocking start() -> DEADLOCK

		// Start NetExecutor if necessary
		if (!netExecutor)
		{
			netExecutor = new NetExecutor(name, netFrequency, isRealtime);
			Registry::getRegistry()->addNetExecutor(netExecutor);
			netExecutor->configure();
			netExecutor->start();
		}
		netExecutor->setCurrentNet(this);
		netState = RPI::Running;
		return true;
	}

	RTT::os::TimeService::nsecs Net::getTime() const
	{
		if (netExecutor)
			return netExecutor->getTime();
		else
			return 0;
	}

	bool Net::acquireResources()
	{

		bool success = true;

		// check availability of all necessary modules
		if (!rootFragment->checkResources())
			return false;

		// lock all resources (state of resources cannot have changed since we still own Mutex)
		rootFragment->acquireResources();

		return true;
	}

	// external registry method
	bool Net::schedule(Net* takeovernet)
	{
		if (netState != Ready)
			return false;

		// lock, basically prevents unloading a net meanwhile
		RTT::os::MutexLock schedulelock(ResourceManager::getResourceManager()->scheduleNetMutex);

		// takeovernet null pointer, or takeovernet already scheduled for cleanup? Immediately start this net
		if (!takeovernet || takeovernet->unloadFlag)
		{
			this->start();
			return true;
		}

		// takeovernet must not be rejected
		if (takeovernet->netState == Rejected)
			return false;

		// other net seems to have terminated abnormally, do not schedule on this net
		if (takeovernet->netState == Terminated && takeovernet->netErrorState.size() > 0)
			return false;

		// only one net can be scheduled to take over another net
		if (takeovernet->scheduledNet)
			return false;

		// scheduled nets must share frequency and RT settings
		if (takeovernet->netFrequency != netFrequency || takeovernet->isRealtime != isRealtime)
			return false;

		// only lock resources when necessary
		{
			RTT::os::MutexLock lock(ResourceManager::getResourceManager()->resourceMutex);

			// check all required resources
			set<string> resources = getRequiredResources();
			set<string> takeresources = takeovernet->getRequiredResources();
			for (set<string>::iterator it = resources.begin(); it != resources.end(); ++it)
			{
				string resourcename = *it;
				// resource must either be unused at the moment or included in the resources requested by previous net
				if ((ResourceManager::getResourceManager()->getResource(resourcename) != "")
						&& (!takeresources.count(resourcename)))
					return false;
			}

			for (set<string>::iterator it = resources.begin(); it != resources.end(); ++it)
			{
				string resourcename = *it;
				// take all resources not required by previous net
				if (!takeresources.count(resourcename))
					ResourceManager::getResourceManager()->addResource(resourcename, this->getName());
			}
		}

		takeovernet->scheduledNet = this;
		this->scheduledByNet = takeovernet;
		this->netState = Scheduled;

		// double start should not hurt
		if (takeovernet->netState == Terminated)
			this->start();

		return true;
	}

	// the parent net has been aborted, clean up
	// must be called within Mutex
	void Net::ParentAborted()
	{
		if (scheduledNet)
		{
			scheduledNet->ParentAborted();
			scheduledNet = 0;
		}
		scheduledByNet = 0;
		configurationError(ParentNetAborted, "Parent net has been aborted");
		netState = Rejected;
	}

	// called only inside Net!
	void Net::stop()
	{
		if (netExecutor)
		{
			netExecutor->setCurrentNet(0);
		}

		{
			// Lock on resource management Mutex
			RTT::os::MutexLock lock(ResourceManager::getResourceManager()->resourceMutex);

			rootFragment->releaseResources();

			RTT::os::MutexLock schedlock(ResourceManager::getResourceManager()->scheduleNetMutex);
			if (!noScheduledNetStart && scheduledNet)
				scheduledNet->acquireResources();

		}

		{
			RTT::os::MutexLock lock(ResourceManager::getResourceManager()->scheduleNetMutex);

			if (scheduledNet)
			{
				// scheduled net will overtake our executor
				if (!noScheduledNetStart)
				{
					scheduledNet->setNetExecutor(netExecutor);
					netExecutor = 0;
					scheduledNet->start();
				} else
				{
					// net was aborted, kill scheduled net
					scheduledNet->ParentAborted();
					scheduledNet = 0;
				}
			}
		}

		if (netExecutor)
		{
			// clean up executor
			netExecutor->finish();
			netExecutor = 0;
		}

		netState = Terminated;
	}

	void Net::cleanup()
	{
		if (rootFragment)
			rootFragment->cleanup();
		if (terminationModule)
			terminationModule->cleanup();
	}

	void Net::setNetExecutor(NetExecutor* executor)
	{
		this->netExecutor = executor;
	}

	NetState Net::getNetState() const
	{
		return netState;
	}

	set<string> Net::getRequiredResources() const
	{
		if(rootFragment)
			return rootFragment->getCachedResourceNames();
		else
			return set<string>();
	}

	// external registry method
	void Net::abort()
	{
		// only running or canceled net can be aborted
		if (netState != RPI::Running && netState != RPI::Cancelling)
			return;

		// do not run scheduled net after abort
		noScheduledNetStart = true;
		stop();
	}

	// external registry method
	void Net::cancel()
	{
		if (netState == RPI::Running || netState == RPI::Scheduled)
			for (std::list<Cancel*>::iterator it = cancelModules.begin(); it != cancelModules.end(); ++it)
				(*it)->setCancel(true);
	}

	// external registry method
	void Net::unload()
	{

		Logger::log() << Logger::Info << getName() << " cycle time:     " << (minCycleTime / 1e6) << " to "
				<< (maxCycleTime / 1e6) << " ms" << RTT::endlog();
		if(currentCycle > 1)
		{
			Logger::log() << Logger::Info << getName() << " average cycle time:     "  << ((avgCycleTimeSum/(currentCycle-1)) / 1e6) << " ms" << RTT::endlog();
		}
		Logger::log() << Logger::Info << getName() << " execution time: " << (minTime / 1e6) << " to "
				<< (maxTime / 1e6) << " ms" << RTT::endlog();

		Logger::log() << Logger::Info << getName() << " primitives count: " << rootFragment->getLeafModules().size() << RTT::endlog();

		// lock mutex (no unloading while anything concerning scheduled nets is performed
		RTT::os::MutexLock lock(ResourceManager::getResourceManager()->scheduleNetMutex);

		if (netState == Terminated || netState == Rejected || netState == Ready || netState == Scheduled)
		{

			// net was scheduled by other net, remove pointer
			if (scheduledByNet)
			{
				scheduledByNet->scheduledNet = 0;
				scheduledByNet = 0;
			}

			// another net was scheduled for execution after this net.
			if (scheduledNet)
			{
				// other net was still scheduled - now rejected
				if (scheduledNet->netState == Scheduled)
					scheduledNet->ParentAborted();
				// remove pointers
				scheduledNet->scheduledByNet = 0;
				scheduledNet = 0;
			}

			/*communicationInProperties.clear();
			 communicationOutProperties.clear();*/

			unloadFlag = true;
			netState = Unloading;
		}
	}

	bool Net::getUnloadFlag() const
	{
		// do not unload if NetExecutor is currently doing something
		return unloadFlag && !currentlyRunning;
	}

	Session_ID_t Net::getSessionId() const
	{
		return session;
	}

	string Net::getName() const
	{
		return name;
	}

	string Net::getDescription()
	{
		return desc;
	}

	string Net::getNetXML()
	{
		return netfragment.toXML();
	}

	string Net::getNetDIO()
	{
		return netfragment.toDIO();
	}

	void Net::configurationError(NetConfigurationError error, const string& message, const string& block)
	{
		NetErrorState nes;
		nes.errorMajor = ConfigurationError;
		nes.errorMinor = error;
		nes.errorMessage = message;
		nes.errorBlock = block;
		netErrorState.push_back(nes);
	}

	void Net::addCommunicationInProperty(const string& name, Netcomm_In* property)
	{
		list<Netcomm_In*>& nclist = communicationInProperties[name];
		nclist.push_back(property);
		communicationData[name] = property->getNetcommData();
	}

	void Net::removeCommunicationInProperty(const string& name)
	{
		communicationInProperties.erase(name);
	}

	map<string, list<Netcomm_In*> > Net::getCommunicationInProperties() const
	{
		return communicationInProperties;
	}

	void Net::addCommunicationOutProperty(const string& name, Netcomm_Out* property)
	{
		communicationOutProperties[name] = property;
		communicationData[name] = property->getNetcommData();
	}

	void Net::removeCommunicationOutProperty(const string& name)
	{
		communicationOutProperties.erase(name);
	}

	map<string, Netcomm_Out*> Net::getCommunicationOutProperties() const
	{
		return communicationOutProperties;
	}

	map<string, string> Net::readCommunicationInProperties() const
	{
		// lock mutex (no unloading while anything concerning scheduled nets is performed
		RTT::os::MutexLock lock(ResourceManager::getResourceManager()->scheduleNetMutex);

		map<string, string> ret;
		for (map<string, list<Netcomm_In*> >::const_iterator it = communicationInProperties.begin();
				it != communicationInProperties.end(); ++it)
		{
			// read value from first Netcomm block, all should have same value
			if (it->second.size() > 0)
				ret[it->first] = it->second.front()->getValue();
		}
		return ret;
	}
	void Net::writeCommunicationInProperties(const map<string, string>& values)
	{
		// lock mutex (no unloading while anything concerning scheduled nets is performed
		RTT::os::MutexLock lock(ResourceManager::getResourceManager()->scheduleNetMutex);

		for (map<string, string>::const_iterator it = values.begin(); it != values.end(); ++it)
		{
			if (communicationInProperties.find(it->first) == communicationInProperties.end())
				continue;
			list<Netcomm_In*> ncinlist = communicationInProperties[it->first];
			for (list<Netcomm_In*>::const_iterator ncit = ncinlist.begin(); ncit != ncinlist.end(); ++ncit)
			{
				Netcomm_In* netcomm = *ncit;
				if (netcomm != NULL)
					netcomm->setValue(it->second);
				else
					RTT::log() << "Illegal netcomm parameter " << it->first << RTT::endlog();
			}
		}
	}
	Netcomm_Map_t Net::readCommunicationOutProperties() const
	{
		// lock mutex (no unloading while anything concerning scheduled nets is performed
		RTT::os::MutexLock lock(ResourceManager::getResourceManager()->scheduleNetMutex);

		Netcomm_Map_t ret;
		for (map<string, Netcomm_Out*>::const_iterator it = communicationOutProperties.begin();
				it != communicationOutProperties.end(); ++it)
		{
			if (it->second->isInitialized())
				ret[it->first] = pair<string, RTT::os::TimeService::nsecs>(it->second->getValue(),
						it->second->getLastValueChange());
		}
		return ret;
	}

	std::shared_ptr<NetcommData> Net::getCommunicationData(const std::string& key) const
	{
		auto it = communicationData.find(key);
		if (it != communicationData.end())
			return it->second;

		return 0;
	}

	void Net::addCancelModule(Cancel* cancelModule)
	{
		cancelModules.push_back(cancelModule);
	}

	void Net::addTakeoverModule(Takeover* takeoverModule)
	{
		takeoverModules.push_back(takeoverModule);
	}

	void Net::addDebugModule(DebugModule* debugModule)
	{
		debug_modules.push_back(debugModule);
	}

	vector<Module*> Net::getModules() const
	{
		vector<Module*> modules;
		modules.push_back(rootFragment);
		modules.push_back(terminationModule);
		return modules;
	}

	vector<DebugModule*> Net::getDebugModules() const
	{
		return debug_modules;
	}

	long Net::getCurrentCycle() const
	{
		return currentCycle;
	}

	double Net::getNetFrequency() const
	{
		return netFrequency;
	}

	bool Net::isRealtimeNet() const
	{
		return isRealtime;
	}

	void Net::processCrashDump()
	{
		if(crashDumpRequested) {
			CrashDump::writeCrashDump(getName(), getCurrentCycle(), getTime(), getNetFrequency()*1e9, getDebugModules());
			std::list<CrashDumper*> timedumpers;
			timedumpers.push_back(executionTimeDumper);
			timedumpers.push_back(executionCycleDumper);
			timedumpers.push_back(actuatorTimeDumper);
			timedumpers.push_back(totalTimeDumper);
			CrashDump::writeCrashDump(getName() + "time", timedumpers);
			crashDumpRequested = false;
		}
	}

	void Net::triggerCrashDump()
	{
		crashDumpRequested = true;
	}

}
