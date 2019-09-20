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
#include <rtt/Logger.hpp>
#include <queue>
#include "Fragment.hpp"
#include "NetExecutorLookup.hpp"
#include "SynchronizationRule.hpp"
#include "ResourceManager.hpp"
#include <cstdarg>
#include "CrashDump.hpp"
// usually not defined in core but in rpicore!
#include <templates/TNetcomm.hpp>
#include "RoundRobinLog.hpp"

namespace RPI
{
	using namespace std;
	using namespace RTT;
	using namespace RTT::os;

	Net::Net(const string& name, const string& desc, ModuleFactoryMap* factory, Session_ID_t session,
			const rpi_fragment& netfragment, double frequency, bool isRealtime) :
			maxTime(0), minTime(TimeService::InfiniteNSecs), maxCycleTime(0), minCycleTime(TimeService::InfiniteNSecs), preModules(), resourceUsagePtr(
					0), session(session), last(0), timeoverrun(0), currentCycle(0), name(name), desc(desc), netFrequency(
					frequency), isRealtime(isRealtime), stopExecution(false), avgCycleTimeSum(0)
	{
		//this->netxml = netxml;
		this->netfragment = netfragment;

		factoryMap = factory;
		terminationState = 0;
		rootFragment = 0;

		unloadFlag = false;
		currentlyRunning = false;
		crashDumpRequested = false;

		resourceUsagePtr = 0;
		resourceUsageCount = 0;

		netExecutor = 0;

		netCommMutex = make_shared<RTT::os::Mutex>();

		// Load the RPI net specified in "net"
		netState = Loading;

		conditionsChanged = false;

		executionTimeDumper = new RoundRobinLog<RTT::os::TimeService::nsecs>();
		executionTimeDumper->setName("executionTime");
		actuatorTimeDumper = new RoundRobinLog<RTT::os::TimeService::nsecs>();
		actuatorTimeDumper->setName("actuatorTime");
		totalTimeDumper = new RoundRobinLog<RTT::os::TimeService::nsecs>();
		totalTimeDumper->setName("totalTime");
		executionCycleDumper = new RoundRobinLog<long>();
		executionCycleDumper->setName("cycle");
		rpiTimeDumper = new RoundRobinLog<RTT::os::TimeService::nsecs>();
		rpiTimeDumper->setName("rpiTime");
	}

	Net::~Net()
	{
//		RTT::log() << name << " removed" << RTT::endlog();

		// release any resources that may have been held
		ResourceManager::getResourceManager()->removeNetUsage(this->name);

		communicationOutProperties.clear();
		communicationInProperties.clear();

		// Unload root fragment and termination primitive
		delete rootFragment;

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
		delete rpiTimeDumper;
	}

	// Must return true on normal runs, and false on aborted (scheduled) runs
	// If true is returned, the update method will only be called in the next cycle
	// If false is returned and another net has been registered with the executor, the
	// update() method of the other net will be called within the SAME cycle again.
	bool Net::update()
	{
		// "lock" unloading of net
		currentlyRunning = true;
		if (unloadFlag)
		{
			currentlyRunning = false;
			return true;
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
		rpiTimeDumper->put(getTime(), tAfterExec);

		for (vector<Module*>::iterator ti = preModules.begin(); ti != preModules.end(); ++ti)
		{
			(*ti)->updatePre();
		}

		// did this execution cycle change any scheduling rule?
		bool schedValuesChanged = false;

		if (netExecutor != 0)
		{
			// lock mutex to prevent actuators writing while scheduling rules are evaluated
			RTT::os::MutexLock actuatorMutex(ResourceManager::getResourceManager()->actuatorMutex);

			if (stopExecution)
				return false;

			// write actuator values
			for (vector<DeviceInstanceT<Device> >::iterator it = usedDevices.begin(); it != usedDevices.end(); ++it)
				(*it)->lockDevice();

			rootFragment->updateActuator();

			for (vector<DeviceInstanceT<Device> >::iterator it = usedDevices.begin(); it != usedDevices.end(); ++it)
				(*it)->unlockDevice();

			// write Boolean Netcomm values to sched values
			for (const auto& sv : schedValues)
			{
				// set new value, returns true if value has changed since last update
				boost::tribool val;

				val = sv.second.first->isNull() ? boost::indeterminate :
						boost::tribool(sv.second.first->getData().first);

				// only recognize sched values as changed, if there is already a SchedCond connected to
				if (sv.second.second->setValue(val) && sv.second.second->isActive())
				{
					schedValuesChanged = true;
				}
			}
		}

		// Now this method must not return false, otherwise two net cycles will be run at once.

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

		currentlyRunning = false;

		// Acquire global lock before scheduling decision
		{
			RTT::os::MutexLock actuatorMutex(ResourceManager::getResourceManager()->actuatorMutex);

			// Check scheduling conditions, only if something has changed in this net
			if (schedValuesChanged || conditionsChanged)
			{
				conditionsChanged = false;

				for (const auto& cond : conditions)
				{
					// Condition fired
					if (cond->CheckCondition())
					{
						if(scheduleCondition(cond))
						{
							Registry::getRegistry()->setSyncState(cond->getSyncID(), SynchronizationRuleState::Fired);
						} else {
							Registry::getRegistry()->setSyncState(cond->getSyncID(), SynchronizationRuleState::FireFailed);
						}
					}
				}
			}

			// Did we terminate on ourself?
			if (netExecutor && terminationState && *terminationState)
			{
//				RTT::log(RTT::LoggerLevel::Info) << "Stopping net " << getName() << " because it terminated." << RTT::endlog();
				stop();
			}
		}

		return true;
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
			terminationState = (bool*) fragment->ports()->getSinglePort("outTerminate")->getValuePtr();
		} else
		{
			configurationError(NoTermination, "Missing outTerminate port on root fragment");
			return false;
		}

		if (!fragment->configure())
		{
			return false;
		}

		// Fetch all boolean Netcomm Out primitives for scheduling and create SchedValues
		for (const auto& ncout : communicationOutProperties)
		{
			// Check whether type matches
			TNetcomm_Out<bool>* boolean_ncout = dynamic_cast<TNetcomm_Out<bool>*>(ncout.second);
			if (boolean_ncout)
			{
				schedValues[ncout.first] = make_pair(boolean_ncout->getTNetcommData(),
						make_shared<SchedCond::SchedValue>());
			}
		}

		map<string, bool*> resourceUsage;

		// look up all resources used in this net and create pointertable
		std::vector<Module*> modules = fragment->getLeafModules();
		std::list<std::string> usedDeviceNames;

		for (vector<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			set<string> resources = (*it)->getResourceNames();
			for (set<string>::iterator it2 = resources.begin(); it2 != resources.end(); ++it2)
			{
				resourceUsage[*it2] = 0;

				// prepare resource in ResourceManager
				ResourceManager::getResourceManager()->prepareResource(*it2);

				resourceNames.insert(*it2);
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
			set<string> resources = (*it)->getResourceNames();
			(*it)->allocateResourceArray(resources.size());

			//for (unsigned int i = 0; i < resources.size(); i++)
			int i = 0;
			for (const auto& res : resources)
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

//		RTT::log(RTT::Debug) << "create resources " << RTT::os::TimeService::Instance()->getNSecs(start) / 1e9
//				<< RTT::endlog();
		start = RTT::os::TimeService::Instance()->getNSecs();

		netExecutor = Registry::getRegistry()->getExecutorLookup()->getNetExecutor(this);
		if(netExecutor == 0) {
			RTT::log(RTT::Error) << "netExecutor is null" << RTT::endlog();
		}

		netState = Ready;
		return true;
	}

	RTT::os::TimeService::nsecs Net::getTime() const
	{
		if (netExecutor)
			return netExecutor->getTime();
		else
			return 0;
	}

	// called only inside Net!
	void Net::stop()
	{
		// signal execution stop
		stopExecution = true;

		// Stop executor, but remain pointer to executor, because a condition could
		// still depend on the executor
		if (netExecutor)
		{
			netExecutor->setNextNet(0);
		}

		netExecutor = 0;

		// TODO: RT safe release of NE
		Registry::getRegistry()->getExecutorLookup()->releaseNetExecutor(this);


		// Remove resource usage
		// No mutex required
		// 1. When scheduling, a mutex is already locked, and resources will be reaquired automatically
		// 2. Otherwise, we don't care about the resource anymore. The ResourceManager is thread-safe itself.

		releaseResources();

		netState = Terminated;

		// Check if conditions are still alive
		/*for (const auto& cond : conditions)
		{
			if (!cond->CheckConditionAlive())
			{
				cond->invalidate();
				Registry::getRegistry()->setSyncState(cond->getSyncID(), SynchronizationRuleState::Inactive);
			}
		}*/
	}

	void Net::cleanup()
	{
		Registry::getRegistry()->getExecutorLookup()->releaseNetExecutor(this);

		if (rootFragment)
			rootFragment->cleanup();
		terminationState = 0;
	}

	// external registry method
	void Net::abort()
	{
		// only running or canceled net can be aborted
		//if (!isRequiredState(2, RPI::Running, RPI::Cancelling))
		//	return;

		// prevent entering of actuator writing in update
		RTT::os::MutexLock lock(ResourceManager::getResourceManager()->actuatorMutex);

//		RTT::log(RTT::LoggerLevel::Info) << "Stopping net " << getName() << " because it was aborted" << RTT::endlog();
		stop();
	}

	// external registry method
	void Net::cancel()
	{
		if (isRequiredState(1, RPI::Running))
			for (std::list<Cancel*>::iterator it = cancelModules.begin(); it != cancelModules.end(); ++it)
				(*it)->setCancel(true);
	}

	// external registry method
	void Net::unload()
	{
		Logger::In in("~" + getName());

		Logger::log(Logger::Info) << "Unloading net with " << rootFragment->getLeafModules().size() << " modules after " << currentCycle << " cycles" << RTT::endlog();
		RTT::os::MutexLock unloadLock(*netCommMutex);
		{
			RTT::os::MutexLock lock(ResourceManager::getResourceManager()->actuatorMutex);
			if (isNotState(1, RPI::Running))
			{
				unloadFlag = true;
				netState = Unloading;

				loseResources();
			}
		}

		// TODO: check if working correct
		this->conditions.clear();

		if (currentCycle > 0)
		{
			Logger::log(Logger::Info) << "Net cycle time:     " << (minCycleTime / 1e6) << " to "
					<< (maxCycleTime / 1e6) << " ms" << RTT::endlog();
			Logger::log(Logger::Info) << "Net execution time: " << (minTime / 1e6) << " to "
					<< (maxTime / 1e6) << " ms" << RTT::endlog();
		}
		if (currentCycle > 1)
		{
			Logger::log(Logger::Info) << "Average cycle time: "
					<< ((avgCycleTimeSum / (currentCycle - 1)) / 1e6) << " ms" << RTT::endlog();
		}

	}

	bool Net::getUnloadFlag() const
	{
		// do not unload if NetExecutor is currently doing something
		return unloadFlag && !currentlyRunning;
	}

	const std::set<std::string>& Net::getResourceNames() const
	{
		return resourceNames;
	}

	void Net::releaseResources()
	{
		for (auto it = resourceNames.begin(); it != resourceNames.end(); ++it)
		{
			ResourceManager::getResourceManager()->removeResource(*it, this->name);
		}
	}

	void Net::acquireResources()
	{
		ResourceManager* rm = ResourceManager::getResourceManager();
		for (const auto& res : resourceNames)
		{
			const auto& wr = rm->getWeakResource(res);
			if (wr != ResourceManager::empty)
			{
				// notify nets of resource loss
				const auto othernet = Registry::getRegistry()->getNet(wr);
				if(othernet)
					othernet->loseResources();
			}
			ResourceManager::getResourceManager()->addResource(res, this->name);
		}
	}

	void Net::loseResources()
	{
		for (auto& sv : schedValues)
			sv.second.second->setValue(boost::indeterminate);
	}

	NetExecutor* Net::getNetExecutor() const
	{
		return netExecutor;
	}

	void Net::setNetExecutor(NetExecutor* executor)
	{
		this->netExecutor = executor;
	}

	NetState Net::getNetState() const
	{
		return netState;
	}

	void Net::setNetState(NetState newstate)
	{
		netState = newstate;
	}

	bool Net::isRequiredState(int n...) const
	{
		va_list va;

		va_start(va, n);

		bool ok = false;

		for(int i = 0; i < n; ++i)
		{
			if(this->netState == ((RPI::NetState) va_arg(va, int)))
			{
				ok = true;
				break;
			}
		}

		va_end(va);

		return ok;
	}

	bool Net::isNotState(int n...) const
	{
		va_list va;

		va_start(va, n);

		bool ok = true;

		for(int i = 0; i < n; ++i)
		{
			if(this->netState == ((RPI::NetState) va_arg(va, int)))
			{
				ok = false;
				break;
			}
		}

		va_end(va);

		return ok;
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

	void Net::addCommunicationOutProperty(const string& name, Netcomm_Out* property, bool report)
	{
		communicationData[name] = property->getNetcommData();
		if(report)
			communicationOutProperties[name] = property;
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
		// lock mutex (no unloading)
		RTT::os::MutexLock lock(netUnloadMutex);

		if (unloadFlag)
			return map<string, string>();

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
		// lock mutex (no unloading)
		RTT::os::MutexLock lock(netUnloadMutex);

		if (unloadFlag)
			return;

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
		// lock mutex (no unloading)
		RTT::os::MutexLock lock(netUnloadMutex);

		if (unloadFlag)
			return Netcomm_Map_t();

		Netcomm_Map_t ret;
		for (map<string, Netcomm_Out*>::const_iterator it = communicationOutProperties.begin();
				it != communicationOutProperties.end(); ++it)
		{
			if (it->second != 0 && it->second->isInitialized())
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

	void Net::addDebugModule(DebugModule* debugModule)
	{
		debug_modules.push_back(debugModule);
	}

	vector<Module*> Net::getModules() const
	{
		vector<Module*> modules;
		modules.push_back(rootFragment);
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

	string Net::getNetDebug()
	{
		int startcycle = 0;
		return getNetDebug(startcycle);
	}

	string Net::getNetDebug(int& startCycle)
	{
		vector<DebugModule*> debugs = this->getDebugModules();
		stringstream ret;
		if (!debugs.empty())
		{
			long end = this->getCurrentCycle();
			long start = end;
			if (startCycle == 0)
				ret << "Tick";
			for (unsigned int i = 0; i < debugs.size(); i++)
			{
				DebugModule* mod = debugs[i];
				if (startCycle == 0)
					ret << "\t" << mod->getName();
				int count = mod->getCount();
				if (end - count < start)
					start = end - count;
			}
			if (startCycle == 0)
				ret << "\n";
			if (start < 0)
				start = 0;
			if (start < startCycle)
				start = startCycle;
			for (long i = start; i < end; i++)
			{
				ret << i * this->getNetFrequency();
				for (unsigned int j = 0; j < debugs.size(); j++)
				{
					DebugModule* mod = debugs[j];
					ret << "\t";
					mod->output(ret, i);
				}
				ret << "\n";
			}
			startCycle = end;
		}
		return ret.str();
	}

	map<string, RTT::os::TimeService::nsecs> Net::getNetBlockMinTime()
	{
		map<string, RTT::os::TimeService::nsecs> ret;
		vector<Module*> modules = this->getModules();
		for (vector<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			ret[(*it)->getName()] = (*it)->minExecutionTime;
		}

		return ret;
	}

	map<string, RTT::os::TimeService::nsecs> Net::getNetBlockMaxTime()
	{
		map<string, RTT::os::TimeService::nsecs> ret;
		vector<Module*> modules = this->getModules();
		for (vector<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			ret[(*it)->getName()] = (*it)->maxExecutionTime;
		}
		return ret;
	}

	map<string, RTT::os::TimeService::nsecs> Net::getNetBlockTotalTime()
	{
		map<string, RTT::os::TimeService::nsecs> ret;
		vector<Module*> modules = this->getModules();
		for (vector<Module*>::iterator it = modules.begin(); it != modules.end(); ++it)
		{
			ret[(*it)->getName()] = (*it)->totalExecutionTime;
		}
		return ret;
	}

	void Net::processCrashDump()
	{
		if (crashDumpRequested)
		{
			CrashDump::writeCrashDump(getName(), getCurrentCycle(), getTime(), getNetFrequency() * 1e9,
					getDebugModules());
			std::list<CrashDumper*> timedumpers;
			timedumpers.push_back(executionTimeDumper);
			timedumpers.push_back(executionCycleDumper);
			timedumpers.push_back(actuatorTimeDumper);
			timedumpers.push_back(totalTimeDumper);
			timedumpers.push_back(rpiTimeDumper);
			CrashDump::writeCrashDump(getName() + "time", timedumpers);
			crashDumpRequested = false;
		}
	}

	void Net::triggerCrashDump()
	{
		crashDumpRequested = true;
	}

	std::shared_ptr<SchedCond::SchedValue> Net::getSchedValue(const std::string& ncname) const
	{
		auto it = schedValues.find(ncname);
		if (it != schedValues.end())
			return it->second.second;

		return 0;
	}

	std::list<std::shared_ptr<SynchronizationRule>> Net::getConditions() const
	{
		return this->conditions;
	}

	void Net::addCondition(shared_ptr<SynchronizationRule> cond)
	{
		this->conditions.push_front(cond);
		conditionsChanged = true;
	}

	bool Net::scheduleCondition(shared_ptr<SynchronizationRule> cond)
	{
		// Mark condition as fired so it won't be examined again
		cond->fired();

		// gather and distribute executors
		const auto& stopnets = cond->getStopNets();
		const auto& cancelnets = cond->getCancelNets();
		const auto& startnets = cond->getStartNets();

		// Check resources
		// NB: we can assume that RHS resource requirements are disjoint

		// check for resources
		const auto& startResources = cond->getStartResources();
		const auto& stopResources = cond->getStopResources();

		for (auto it = startResources.begin(); it != startResources.end(); ++it)
		{
			auto owner = ResourceManager::getResourceManager()->getResource(*it);
			auto preowner = ResourceManager::getResourceManager()->getWeakResource(*it);

			// No one owns resource
			if (owner == ResourceManager::empty)
			{
				/*// No stop net ever required this resource -> everything is fine
				 if (stopResources.find(*it) == stopResources.end())
				 {
				 continue;
				 } else
				 {
				 // Any stop net is still preowner (i.e. no one else has taken resource meanwhile)
				 if (cond->hasStopNet(preowner))
				 {
				 continue;
				 }
				 }*/
				continue;
			}

			// StopNet owns resource -> ok
			if (cond->hasStopNet(owner))
				continue;

			// Someone else owns resource, this must not happen!
			cond->invalidate();
			return false;
		}

		// check if startnets are in right state
		for (const auto& net : startnets)
		{
			// check whether start net is not running
			if (!net->isRequiredState(2, RPI::Ready, RPI::Scheduled))
			{
				cond->invalidate();
				return false;
			}
		}

		// ************************************
		// from now on scheduling must not fail
		// ************************************

		// Cancel nets
		for (const auto& net : cancelnets)
		{
			net->cancel();
		}

		// Stop nets
		for (const auto& net : stopnets)
		{
			NetExecutor* ne = net->getNetExecutor();
			if (ne)
			{
				// commented out: unnecessary - is done in stop

				// Disable executor (temporarily)
				//ne->setNextNet(0);
				// Release resources (will potentially be reclaimed during this scheduling
				// session)

				//net->releaseResources();
				// Stop Net

//				RTT::log(RTT::LoggerLevel::Info) << "Stopping net " << net->getName() << " for rule " << cond->getSyncID() << RTT::endlog();
				net->stop();

				// remove executor
				net->setNetExecutor(0);
			}
		}

		for (auto it = startnets.begin(); it != startnets.end(); ++it)
		{
			auto net = *it;
			net->getNetExecutor()->setNextNet(net.get());

			net->setNetState(RPI::Running);
			// lock resources
			net->acquireResources();
		}

		cond->invalidate();
		return true;
	}

	bool Net::checkResourcesForExecutor(const std::set<std::string>& providedResourceNames,
			const std::set<std::string>& necessaryResources)
	{
		for (auto it = necessaryResources.begin(); it != necessaryResources.end(); ++it)
		{
			if (providedResourceNames.find(*it) != providedResourceNames.end())
				return true;
		}
		return false;
	}

	bool Net::scheduleNets(Sync_ID_t id, std::unique_ptr<SchedCond::SchedCondition> schedcondition, std::set<Net_ID_t> stopnetids,
			std::set<Net_ID_t> cancelnetids, std::set<Net_ID_t> startnetids, const std::string& description)
	{
		set<string> requiredResources;
		set<shared_ptr<Net>> startNets, cancelNets, stopNets;

		// verify all start nets are valid and add to new condition
		for (const auto& netid : startnetids)
		{
			auto net = Registry::getRegistry()->getNet(netid);

			// specified net not found
			if (!net)
				return false;

			// net must be ready or already scheduled
			if (!net->isRequiredState(2, RPI::Ready, RPI::Scheduled, RPI::Running))
				return false;

			// identify resources required for this net
			set<string> resourceNames = net->getResourceNames();
			for (auto it2 = resourceNames.begin(); it2 != resourceNames.end(); ++it2)
			{
				if (requiredResources.find(*it2) == requiredResources.end())
				{
					// Resource not required by other RHS net
					requiredResources.insert(*it2);
				} else
				{
					// Resource already required by other RHS net - this is not allowed
					return false;
				}
			}

			startNets.insert(net);
		}

		// verify all stop nets are valid and add to new condition
		for (const auto& netid : stopnetids)
		{
			auto net = Registry::getRegistry()->getNet(netid);

			if (!net)
				return false;

			if (!net->isRequiredState(4, RPI::Ready, RPI::Running, RPI::Scheduled, RPI::Terminated))
				return false;

			stopNets.insert(net);
		}

		// verify all cancel nets are valid and add to new condition
		for (const auto& netid : cancelnetids)
		{
			auto net = Registry::getRegistry()->getNet(netid);

			if (!net)
				return false;

			if (!net->isRequiredState(4, RPI::Ready, RPI::Running, RPI::Scheduled, RPI::Terminated))
				return false;

			cancelNets.insert(net);
		}

		auto condnets = schedcondition->getCondNets();

		for (const auto& netid : condnets)
		{
			auto net = Registry::getRegistry()->getNet(netid);

			if (!net)
				return false;

			if (!net->isRequiredState(4, RPI::Ready, RPI::Running, RPI::Scheduled, RPI::Terminated))
				return false;
		}

		try
		{

			shared_ptr<SynchronizationRule> cond(
					new SynchronizationRule(id, std::move(schedcondition), stopNets, cancelNets, startNets, description));
			// schedcondition MUST NOT BE USED from here - it's a null pointer!

			// This Mutex prevents any changes in nets, i.e. no scheduling is performed
			// while this mutex is locked. Locking time should be as short as possible
			RTT::os::MutexLock globalMutex(ResourceManager::getResourceManager()->actuatorMutex);

			for (const auto& net : cond->getConditionNets())
			{
				net->addCondition(cond);
			}

			for (const auto& net : cond->getStartNets())
			{
				net->setNetState(RPI::Scheduled);
			}

			// Check whether condition is alive
			if (!cond->CheckConditionAlive())
			{
				// All condition nets are already terminated (or no condition exists)
				// Check condition and start (or dispose)

				if (cond->CheckCondition())
				{
					if(Net::scheduleCondition(cond))
					{
						Registry::getRegistry()->setSyncState(cond->getSyncID(), SynchronizationRuleState::Fired);
					} else {
						Registry::getRegistry()->setSyncState(cond->getSyncID(), SynchronizationRuleState::FireFailed);
					}
				} else
				{
					// Condition was not true and will never become true
					Registry::getRegistry()->setSyncState(cond->getSyncID(), SynchronizationRuleState::Inactive);
					cond->invalidate();
				}
			}
		} catch (SynchronizationRuleException&)
		{
			// SynchronizationRule could not be created, just ignore and return false

			return false;
		}

		return true;

	}
}
