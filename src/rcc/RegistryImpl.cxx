/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <set>
#include <vector>
#include <algorithm>
#include <rtt/Logger.hpp>
#include "RegistryImpl.hpp"
#include "Server/HTTPHandlers.hpp"
#include "NetModules.hpp"
#include "VirtualRCCDevice.hpp"
#include "NetExecutor.hpp"
#include "NetParser.hpp"
#include "Fragment.hpp"
#include "ExtensionLoader.hpp"
#include "Server/DirectIONet/DirectIONet.h"
#include "Server/DirectIONet/DirectIONetConverter.hpp"
#include "Server/websocket/WebsocketStreamHandler.hpp"
#include "ResourceManager.hpp"
#include "CrashDump.hpp"
#include "NetExecutorLookup.hpp"

namespace RPI
{
	using namespace std;

	RegistryImpl::RegistryImpl() :
			RTT::os::Thread(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0.1, 0, "RPIRegistry"),
			rpiNetCounter(0), rpiSyncCounter(0), sessionCounter(1), rpiNets(), vrcc(0)
	{

#ifdef HAVE_REALTIME
		for(unsigned int i = 0; i < max_cpu; ++i)
			cpu_load[i] = 0;

		// penalty for first cpu (devices, etc.)
		cpu_load[0] = 200;
#endif

		// adding Handlers for Web-Interface

		// Net related stuff
		HTTPServer::getInstance()->addHandler("/nets/", new NetsHandler());
		HTTPServer::getInstance()->addHandler("/nets/*/", new NetHandler());
		HTTPServer::getInstance()->addHandler("/nets/*/xml", new NetXmlHandler());
		HTTPServer::getInstance()->addHandler("/nets/*/dio", new NetDioHandler());
		HTTPServer::getInstance()->addHandler("/nets/*/debug", new NetDebugHandler());
		HTTPServer::getInstance()->addHandler("/nets/*/stats", new NetStatsHandler());
		HTTPServer::getInstance()->addStreamHandler("/nets/*/debugstream", new NetDebugStreamHandler());
		HTTPServer::getInstance()->addHandler("/eval/", new EvalHandler());

		// Sessions
		HTTPServer::getInstance()->addHandler("/sessions/", new SessionsHandler());
		HTTPServer::getInstance()->addHandler("/sessions/*/", new SessionHandler());

		// Types
		HTTPServer::getInstance()->addHandler("/types/", new TypesHandler());
		HTTPServer::getInstance()->addHandler("/types/*/", new TypeHandler());
		HTTPServer::getInstance()->addHandler("/types/*/source.java", new TypeSourceHandler());

		// Modules
		HTTPServer::getInstance()->addHandler("/modulesdetails/", new ModulesDetailsHandler());
		HTTPServer::getInstance()->addHandler("/modules/", new ModulesHandler());
		HTTPServer::getInstance()->addHandler("/modules/*/", new ModuleHandler());
		HTTPServer::getInstance()->addHandler("/modules/*/source.java", new ModuleSourceHandler());

		// Drivers, n.b. drivers can add handlers in their load(...) method
		HTTPServer::getInstance()->addHandler("/devices/", new DevicesHandler());
		HTTPServer::getInstance()->addHandler("/devices/rcc/", new RCCDeviceHandler());
		HTTPServer::getInstance()->addHandler("/devices/*/parameters", new DeviceParameterHandler());
		HTTPServer::getInstance()->addHandler("/extensions/", new ExtensionsHandler());

		// Log Handler
		HTTPServer::getInstance()->addHandler("/log/" , new LogHandler());

		HTTPServer::getInstance()->addStreamHandler("/DirectIO/", new WebsocketStreamHandler());

		executorLookup = new NetExecutorLookup();
	}

	RegistryImpl::~RegistryImpl()
	{
		delete executorLookup;
	}

	// Session stuff

	Session_ID_t RegistryImpl::createSessionID(const string& description)
	{
		rpiNetCounterMutex.lock();
		Session_ID_t session = sessionCounter++;
		rpiNetCounterMutex.unlock();

		RTT::os::MutexLock lock(rpiNetListMutex);
		sessions[session] = description;
		return session;
	}
	vector<Session_ID_t> RegistryImpl::listSessions()
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		vector<Session_ID_t> ret;
		for (map<Session_ID_t, string>::iterator ti = sessions.begin(); ti != sessions.end(); ++ti)
		{
			ret.push_back(ti->first);
		}
		return ret;
	}
	string RegistryImpl::getSessionName(Session_ID_t id)
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		if (sessions.find(id) != sessions.end())
		{
			stringstream str;
			str << "s" << id;
			return str.str();
		}
		return "INVALID";
	}
	string RegistryImpl::getSessionDesc(Session_ID_t id)
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		if (sessions.find(id) != sessions.end())
			return sessions[id];
		else
			return "";
	}
	Session_ID_t RegistryImpl::getSessionForName(const string& name)
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		stringstream str;
		str.str(name);
		Session_ID_t session;
		str.get();
		str >> session;
		if (sessions.find(session) != sessions.end())
			return session;
		return SESSION_INVALID;
	}
	vector<Net_ID_t> RegistryImpl::listSessionNets(Session_ID_t id)
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		vector<Net_ID_t> ret;
		for (auto ti = rpiNets.begin(); ti != rpiNets.end(); ++ti)
		{
			if (ti->second->getSessionId() == id)
				ret.push_back(ti->first);
		}
		return ret;
	}
	void RegistryImpl::endSession(Session_ID_t id)
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		sessions.erase(id);
	}

	// End of Session stuff

	Net_ID_t RegistryImpl::createNetID()
	{
		rpiNetCounterMutex.lock();
		Net_ID_t netnum = rpiNetCounter++;
		rpiNetCounterMutex.unlock();
		return netnum;
	}

	bool RegistryImpl::loadNetXML(const string& desc, const string& xml, Net_ID_t netnum, Session_ID_t session, double frequency, bool isRealtime)
	{
		RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();
		rpi_fragment parsednet;
		if(!parseXML(xml, parsednet)) return false;

//		RTT::log(RTT::Debug) << "Registry::parseXML() " << RTT::os::TimeService::Instance()->getNSecs(start) / 1e9 << "s"
//				<< RTT::endlog();
		return loadNet(desc, parsednet, netnum, session, frequency, isRealtime);
	}

	bool RegistryImpl::loadNetDIO(const string& desc, const string& dio, Net_ID_t netnum, Session_ID_t session, std::string& errors, double frequency, bool isRealtime)
	{
		RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();
		rpi_fragment parsednet;
		if(!parseDIO(dio, parsednet)) {
			errors = "Syntax error in net.";
			return false;
		}

//		RTT::log(RTT::Debug) << "Registry::parseXML() " << RTT::os::TimeService::Instance()->getNSecs(start) / 1e9 << "s"
//				<< RTT::endlog();
		return loadNet(desc, parsednet, netnum, session, frequency, isRealtime);
	}

	bool RegistryImpl::parseDIO(const string& dio, rpi_fragment& fragment)
	{
		DirectIONet::Scanner scanner((unsigned char*) dio.c_str(), dio.length());
		DirectIONet::Parser parser(&scanner);
		parser.Parse();
		if (parser.errors->count > 0) {
			RTT::log(RTT::Debug) << "Registry::parseDIO() failed: " << parser.errors->count << " errors." << RTT::endlog();

			// errors = parser.errors->getMessage();
			return false;
		}
		DirectIONetConverter converter;
		fragment = converter.convertNet(parser.finalfrag);
		return true;
	}

	bool RegistryImpl::parseXML(const string& xml, rpi_fragment& fragment)
	{
		try
		{
			fragment = NetParser::parseNet(xml);
			return true;
		} catch (NetParserException&)
		{
			return false;
		}
	}

	std::map<std::string, std::string> RegistryImpl::evalFragmentDIO(std::string dio) {
		std::map<std::string, std::string> ret;
		rpi_fragment parsednet;
		if(!parseDIO(dio, parsednet)) {
			ret["error"] = "Failed to parse net.";
			return ret;
		}
		return evalFragment(parsednet);
	}

	std::map<std::string, std::string> RegistryImpl::evalFragmentXML(std::string dio) {
		std::map<std::string, std::string> ret;
		rpi_fragment parsednet;
		if(!parseXML(dio, parsednet)) {
			ret["error"] = "Failed to parse net.";
			return ret;
		}
		return evalFragment(parsednet);
	}

	std::map<std::string, std::string> RegistryImpl::evalFragment(rpi_fragment& fragment) {
		std::map<std::string, std::string> ret;
		rpi_fragment empty;
		auto netid = createNetID();
		stringstream netname;
		netname << "eval" << netid;
		Net* net = new Net(netname.str(), "EvalNet", ExtensionLoader::getInstance()->getModuleFactoryMap(), 0, empty, 0.001, false);
		Fragment* frag = new Fragment(net,  ExtensionLoader::getInstance()->getModuleFactoryMap(), fragment);
		if(!frag->buildFragment()) {
			ret["error"] = "Failed to build fragment.";
			delete frag;
			delete net;
			return ret;
		}
		if(!frag->configure()) {
			ret["error"] = "Failed to configure fragment.";
			delete frag;
			delete net;
			return ret;
		}


		auto leaf = frag->getLeafModules();
		for(auto li = leaf.begin(); li != leaf.end(); ++li) {
			if((*li)->isSensor())
				(*li)->updateSensor();
		}
		frag->updateHook();

		std::vector<std::string> ports = frag->ports()->getPortNames();
		for(auto it = ports.begin(); it != ports.end(); ++it) {
			Port* port = frag->ports()->getFirstPort(*it);
			if(port->isOutPort() && *port->getLastWriteCyclePtr() != -1) {
				ret[*it] = port->getType()->toString(port->getValuePtr());
			}
		}

		Netcomm_Map_t netcomm = net->readCommunicationOutProperties();
		for(auto it = netcomm.begin(); it != netcomm.end(); ++it) {
			ret[it->first] = it->second.first;
		}

		delete frag;
		delete net;
		return ret;
	}

	bool RegistryImpl::loadNet(const string& desc, const rpi_fragment& fragment, Net_ID_t netnum, Session_ID_t session, double frequency, bool isRealtime)
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();
		stringstream netname;
		netname << "rpinet" << netnum;
		RTT::Logger::In in(netname.str());

		RTT::log(RTT::Info) << "Creating " << netname.str() << " for " << desc << RTT::endlog();

		shared_ptr<Net> newnet(new Net(netname.str(), desc, ExtensionLoader::getInstance()->getModuleFactoryMap(), session, fragment, frequency, isRealtime));

//		RTT::log(RTT::Debug) << "Net::ctor() " << RTT::os::TimeService::Instance()->getNSecs(start) / 1e9 << "s" << RTT::endlog();
		start = RTT::os::TimeService::Instance()->getNSecs();

		newnet->configure();

//		RTT::log(RTT::Debug) << "Net::configure() " << RTT::os::TimeService::Instance()->getNSecs(start) / 1e9 << "s" << RTT::endlog();
		start = RTT::os::TimeService::Instance()->getNSecs();

		rpiNets[netnum] = newnet;

		return true;

	}

	Net_ID_t RegistryImpl::getNetID(const string& name) const
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		for (auto ti = rpiNets.begin(); ti != rpiNets.end(); ++ti)
		{
			if (ti->second != 0 && ti->second->getName() == name)
			{
				return ti->first;
			}
		}
		return NET_INVALID;
	}

	inline std::shared_ptr<Net> RegistryImpl::getNet(Net_ID_t netID) const
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		auto it = rpiNets.find(netID);

		if (it != rpiNets.end())
			return it->second;
		else
			return 0;
	}

	inline std::shared_ptr<Net> RegistryImpl::getNet(const string& name) const
	{
		return getNet(getNetID(name));
	}

	vector<Net_ID_t> RegistryImpl::listNets()
	{
		RTT::os::MutexLock lock(rpiNetListMutex);
		vector<Net_ID_t> ret;
		for (auto ti = rpiNets.begin(); ti != rpiNets.end(); ++ti)
		{
			ret.push_back(ti->first);
		}
		return ret;
	}

	void RegistryImpl::addNetExecutor(NetExecutor* executor)
	{
		RTT::os::MutexLock lock(executorMutex);
		executors.push_back(executor);
	}

	void RegistryImpl::killAllNets()
	{
		RTT::log(RTT::Info) << "Killing all nets ..." << RTT::endlog();
		vector<int> nets = listNets();
		for (vector<int>::iterator it = nets.begin(); it != nets.end(); ++it)
		{
			auto net = getNet(*it);
			net->abort();
			net->unload();
		}
		RTT::log(RTT::Info) << "All nets aborted and unloaded" << RTT::endlog();
	}


	///////////////////////////////////////////////////////////////////////////////
	// DEVICE STUFF

	/**
	 * fetches pointer to device without registering device usage
	 * \param name Name of device
	 * \return pointer to device, might get invalid immediately
	 * \deprecated
	 */
	Device* RegistryImpl::getDevice(const string& name) const
	{
		RTT::os::MutexLock lock(devicesMutex);
		devices_t::const_iterator it;
		it = devices.find(name);
		if (it != devices.end())
		{
			return it->second.first;
		} else
		{
			return 0;
		}
	}

	/**
	 * fetches pointer to device and registering an owner to the device, preventing device unloading
	 * before owner releases the device
	 * \param name Name of device
	 * \param holder Pointer to holding object \see DeviceInstance
	 * \return pointer to device, remains valid at least until releaseDevice has been called
	 */
	Device* RegistryImpl::getAndAquireDevice(const string& name, DeviceInstance* holder)
	{
		RTT::os::MutexLock lock(devicesMutex);
		devices_t::const_iterator it;
		it = devices.find(name);

		if(it == devices.end())
			return 0;

		set<DeviceInstance*>& holders = usedDeviceInstances[name];
		holders.insert(holder);

		return it->second.first;

	}

	/**
	 * Release named device. After releasing, the device might get freed immediately, therefore
	 * no pointer to the device should be kept.
	 * \param name Name of device
	 * \param holder Pointer to holding object
	 */
	void RegistryImpl::releaseDevice(const string& name, DeviceInstance* holder)
	{
		RTT::os::MutexLock lock(devicesMutex);

		set<DeviceInstance*>& holders = usedDeviceInstances[name];
		holders.erase(holder);
	}


	DeviceFactory* RegistryImpl::getDeviceFactory(const string& type) const
	{
		DeviceFactoryMap* dfm = ExtensionLoader::getInstance()->getDeviceFactoryMap();

		DeviceFactoryMap::iterator dfmit = dfm->find(type);
		if(dfmit == dfm->end())
			return 0;

		return dfmit->second;
	}

	/**
	 * Creates a new device.
	 * \param type Type of device
	 * \param name Desired name of device
	 * \param parameters Parameters for device creation
	 * \return True, if a new device has been created. False if device could not be created
	 * 	e.g. because device with the given name already exists, or if no device of desired type
	 * 	can be created
	 */
	bool RegistryImpl::createDevice(const string& type, const string &name, const parameter_t& parameters)
	{
		if(!vrcc && type == dev_virtualrcc)
		{
			RTT::os::MutexLock lock(devicesMutex);
			vrcc = new VirtualRCCDevice(dev_virtualrcc, parameters);
			devices[dev_virtualrcc] = pair<Device*, string>(vrcc, dev_virtualrcc);
			return true;
		}


		// optimistic creation of device without lock, otherwise no access to other devices possible
		// TODO: If extension unloading is possible, a mutex must prevent the devicefactory to disappear
		DeviceFactoryMap* dfm = ExtensionLoader::getInstance()->getDeviceFactoryMap();

		DeviceFactoryMap::iterator dfmit = dfm->find(type);
		if(dfmit == dfm->end()) {
			RTT::log(RTT::Critical) << "Create Device " << name << " failed, invalid type" << RTT::endlog();
			return false;
		}

		RTT::log(RTT::Info) << "Create Device " << name << " started" << RTT::endlog();

		Device* dev = dfmit->second->createInstance(name, parameters);


		RTT::os::MutexLock lock(devicesMutex);

		// Check if device with that name already exists
		if(devices.find(name) != devices.end())
		{
			dfmit->second->destroyInstance(name, dev);
			RTT::log(RTT::Critical) << "Create Device " << name << " aborted" << RTT::endlog();
			return false;
		}

		RTT::log(RTT::Info) << "Create Device " << name << " finished" << RTT::endlog();


		devices[name] = pair<Device*, string>(dev, type);

		return true;
	}

	void RegistryImpl::removeDevice(const string &name)
	{
		RTT::os::MutexLock lock(devicesMutex);

		devices_t::iterator it = devices.find(name);
		// if there is no device with that name, just do nothing
		if(it == devices.end())
			return;

		// This device does not want to be removed (e.g. rcc dummy driver)
		if(!it->second.first->isRemovable())
			return;

		devices_toremove.insert(pair<string, device_type_t>(it->first, it->second));
		devices.erase(it);
	}

	/**
	 * Returns a list of all currently existing devices
	 * \return Vector of pairs (name, type)
	 */
	devicelist_t RegistryImpl::getDevices() const
	{
		devicelist_t ret;

		RTT::os::MutexLock lock(devicesMutex);

		for(devices_t::const_iterator it = devices.begin(); it != devices.end(); ++it)
		{
			DeviceInfo info;
			info.name = it->first;
			info.type = it->second.second;

			info.implementedInterfaces = it->second.first->implementedDeviceInterfaces;

			ret.push_back(info);
		}

		return ret;
	}

	/**
	 * Returns a list of all currently existing devices
	 * \return Vector of pairs (name, type)
	 */
	RPI::DeviceState RegistryImpl::getDeviceState(std::string name) const
	{
		RTT::os::MutexLock lock(devicesMutex);

		devices_t::const_iterator it = devices.find(name);
		// if there is no device with that name, just do nothing
		if(it == devices.end())
			return RPI::DeviceState::OFFLINE;

		return it->second.first->getDeviceState();

	}

	/**
	 * Returns a list of all currently existing devices
	 * \return Vector of pairs (name, type)
	 */
	RTT::os::TimeService::nsecs RegistryImpl::getDeviceLastUpdated(std::string name) const
	{
		RTT::os::MutexLock lock(devicesMutex);

		devices_t::const_iterator it = devices.find(name);
		// if there is no device with that name, just do nothing
		if(it == devices.end())
			return 0;

		return it->second.first->lastUpdated;

	}


	VirtualRCCDevice* RegistryImpl::getVirtualRCC() const
	{
		return vrcc;
	}


	bool RegistryImpl::initialize()
	{
		ExtensionLoader::getInstance()->loadConfD();

		// no configuration for vrcc?
		if(!vrcc)
		{
			// initialize virtual rcc device
			vrcc = new VirtualRCCDevice("rcc", parameter_t());
			devices[dev_virtualrcc] = pair<Device*, string>(vrcc, "rcc");
		}

		if(!HTTPServer::getInstance()->start(vrcc->getWebPort()))
		{
			RTT::log(RTT::Fatal) << "Required port " << vrcc->getWebPort() << " could not be opened, RCC terminating" << RTT::endlog();
			return false;
		}

		return true;
	}

	void RegistryImpl::step()
	{
		if(vrcc && vrcc->getNetUnloading())
		{
			RTT::os::MutexLock lock(rpiNetListMutex);
			RTT::os::MutexLock execlock(executorMutex);

			// unload nets
			for (auto ti = rpiNets.begin(); ti != rpiNets.end();)
			{
				shared_ptr<Net> knet = ti->second;
				if (knet && knet->getUnloadFlag())
				{
					knet->cleanup();
					rpiNets.erase(ti++);
				} else
				{
					ti++;
				}
			}

			// unload net executors
			for (list<NetExecutor*>::iterator it = executors.begin(); it != executors.end();)
			{
				if ((*it)->isFinished())
				{
					(*it)->stop();

					delete (*it);

					executors.erase(it++);
				} else {
					it++;
				}
			}
		}

		{
			// unload devices
			RTT::os::MutexLock lock(devicesMutex);

			for (devices_remove_t::iterator it = devices_toremove.begin(); it != devices_toremove.end();)
			{
				set<DeviceInstance*>& holders = usedDeviceInstances[it->first];
				if (holders.empty())
				{
					DeviceFactoryMap* dfm = ExtensionLoader::getInstance()->getDeviceFactoryMap();

					DeviceFactoryMap::iterator dfmit = dfm->find(it->second.second);
					// this should not happen, no DeviceFactory for existing device??
					if (dfmit == dfm->end())
						return;

					// Destroy instance, and if successful, remove from list
					dfmit->second->destroyInstance(it->first, it->second.first);

					devices_toremove.erase(it++);

				} else
				{
					it++;
				}
			}
		}

		{
			RTT::os::MutexLock lock(devicesMutex);

			// handle crash dumps
			for(const auto& device: devices) {
				if(device.second.first)
					device.second.first->processCrashDump();
			}
			for(const auto& net: rpiNets) {
				net.second->processCrashDump();
			}
		}

		{
			RTT::os::MutexLock lock(devicesMutex);

			// update all device interfaces that are marked as outdated
			for(const auto& device: devices) {
				if(device.second.first->lastUpdated == 0) {
					DeviceFactoryMap* dfm = ExtensionLoader::getInstance()->getDeviceFactoryMap();
					DeviceFactoryMap::iterator dfmit = dfm->find(device.second.second);
					// this should not happen, no DeviceFactory for existing device??
					if (dfmit == dfm->end())
						continue;

					dfmit->second->updateInterfaces(device.first, device.second.first);
				}
			}

		}
		string logline;
		while((logline = RTT::Logger::Instance()->getLogLine()) != "")
			logEvents.push_back(logline);
	}

	void RegistryImpl::finalize()
	{
		// hold both mutexes, not method calles must try to get one of these Mutexes
		RTT::os::MutexLock locknl(rpiNetListMutex);
		RTT::os::MutexLock lockex(executorMutex);

		// stop vrcc, do not notify drivers any longer
		if(vrcc)
			vrcc->stop();

		// Look for all executors that are currently running and stop these
		for (list<NetExecutor*>::iterator ti = executors.begin(); ti != executors.end(); ++ti)
		{
			NetExecutor* netexecutor = *ti;
			if (netexecutor)
			{
				netexecutor->stop();
			}
		}

		// Look for all nets that are currently running and stop these nets
		for (auto ti = rpiNets.begin(); ti != rpiNets.end(); ++ti)
		{
			auto net = ti->second;
			if (net)
			{
				net->abort();
				net->cleanup();
			}
		}

		// will delete most nets due to destruction of shared_ptrs
		rpiNets.clear();

		// Look for all executors and destroy instances
		for (list<NetExecutor*>::iterator ti = executors.begin(); ti != executors.end(); ++ti)
		{
			delete *ti;
		}

		executors.clear();
		rpiNets.clear();


		RTT::os::MutexLock lock(devicesMutex);

		unsigned int ocount, ncount;
		ncount = devices_toremove.size() + devices.size();
		do
		{
			ocount = ncount;
			// remove all devices
			for (devices_remove_t::iterator it = devices_toremove.begin(); it != devices_toremove.end();)
			{
				set<DeviceInstance*>& holders = usedDeviceInstances[it->first];
				if (holders.empty())
				{
					DeviceFactory* df = getDeviceFactory(it->second.second);
					if (df)
						df->destroyInstance(it->first, it->second.first);

					devices_toremove.erase(it++);
				} else
				{
					++it;
				}
			}

			for (devices_t::iterator it = devices.begin(); it != devices.end();)
			{
				set<DeviceInstance*>& holders = usedDeviceInstances[it->first];
				if (holders.empty())
				{
					DeviceFactory* df = getDeviceFactory(it->second.second);
					if (df)
						df->destroyInstance(it->first, it->second.first);
					devices.erase(it++);
				} else
				{
					++it;
				}
			}
			ncount = devices_toremove.size() + devices.size();
		} while (ncount < ocount);

		delete vrcc;
		vrcc = 0;

		ExtensionLoader::finalize();
	}

	void RegistryImpl::triggerCrashDump()
	{
		RTT::log(RTT::Info) << "Requesting crash dump." << RTT::endlog();
		auto devices = getDevices();
		for(const auto& device: getDevices()) {
			getDevice(device.name)->triggerCrashDump();
		}
		for(const auto& net: rpiNets) {
			net.second->triggerCrashDump();
		}
	}

	RegistryImpl* RegistryImpl::theRegistry = 0;

	RegistryImpl* RegistryImpl::getRegistry()
	{
		if (!theRegistry)
		{
			theRegistry = new RegistryImpl();
		}
		return theRegistry;
	}

	void RegistryImpl::clean()
	{
		delete (theRegistry);
		theRegistry = 0;
	}

	std::list<std::string> RegistryImpl::getLogEvents() const
	{
		return logEvents;
	}

	NetExecutorLookup* RegistryImpl::getExecutorLookup() const
	{
		return executorLookup;
	}

	Sync_ID_t RegistryImpl::createSyncID()
	{
		RTT::os::MutexLock lock(rpiSyncCounterMutex);
		return rpiSyncCounter++;
	}

	SynchronizationRuleState RegistryImpl::getSyncState(Sync_ID_t syncID) const
	{
		RTT::os::MutexLock lock(rpiSyncCounterMutex);
		auto res = syncRuleStates.find(syncID);
		if(res != syncRuleStates.end())
			return res->second;
		else
			return SynchronizationRuleState::InvalidRule;
	}

	void RegistryImpl::setSyncState(Sync_ID_t syncID, SynchronizationRuleState newstate)
	{
		RTT::os::MutexLock lock(rpiSyncCounterMutex);
		syncRuleStates[syncID] = newstate;
	}


	unsigned int RegistryImpl::getNextCPU()
	{
#ifdef HAVE_REALTIME
		unsigned int min_load = numeric_limits<unsigned int>::max();
		unsigned int min = 0;
		for(unsigned int i = 0; i < max_cpu; ++i)
		{
			if(cpu_load[i] < min_load)
			{
				min_load = cpu_load[i];
				min = i;
			}
		}

		cpu_load[min]++;
		return min;
#else
		return 0;
#endif
	}

	void RegistryImpl::releaseCPU(unsigned int cpu)
	{
#ifdef HAVE_REALTIME
		if (cpu > 0 && cpu <= max_cpu)
		{
			if (cpu_load[cpu] > 0)
			{
				cpu_load[cpu]--;
			}
		}
#endif
	}
}
