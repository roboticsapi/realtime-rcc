/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef RPIREGISTRYIMPL_HPP_
#define RPIREGISTRYIMPL_HPP_

#include "Registry.hpp"

#include <string>
#include <rtt/os/TimeService.hpp>
#include <rtt/os/Thread.hpp>
#include "NetFwd.hpp"
#include "DeviceFwd.hpp"
#include "NetAST.hpp"


namespace RPI
{
	class VirtualRCCDevice;

	typedef int Net_ID_t;

	class RTT_EXPORT RegistryImpl: public Registry, public RTT::os::Thread
	{
	public:
		static RegistryImpl* getRegistry();
		static void clean();

		Session_ID_t createSessionID(const std::string& description);
		std::vector<Session_ID_t> listSessions();
		std::string getSessionName(Session_ID_t id);
		std::string getSessionDesc(Session_ID_t id);
		Session_ID_t getSessionForName(const std::string& name);
		std::vector<Net_ID_t> listSessionNets(Session_ID_t id);
		void endSession(Session_ID_t id);

		virtual Net_ID_t createNetID();
		virtual Net_ID_t getNetID(const std::string& name) const;
		virtual std::shared_ptr<Net> getNet(Net_ID_t netID) const;
		virtual std::shared_ptr<Net> getNet(const std::string& name) const;

		std::vector<Net_ID_t> listNets();

		bool loadNetXML(const std::string& desc, const std::string& xml, Net_ID_t id, Session_ID_t session, double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true);
		bool loadNetDIO(const std::string& desc, const std::string& dio, Net_ID_t id, Session_ID_t session, std::string& errors, double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true);
		std::map<std::string, std::string> evalFragmentXML(const std::string xml);
		std::map<std::string, std::string> evalFragmentDIO(const std::string dio);

		bool createDevice(const std::string& type, const std::string& name, const parameter_t& parameters);
		Device* getDevice(const std::string& name) const;
		Device* getAndAquireDevice(const std::string& name, DeviceInstance* holder);
		void releaseDevice(const std::string& name, DeviceInstance* holder);
		void removeDevice(const std::string& name);
		devicelist_t getDevices() const;
		RPI::DeviceState getDeviceState(std::string name) const;
		RTT::os::TimeService::nsecs getDeviceLastUpdated(std::string name) const;
		DeviceFactory* getDeviceFactory(const std::string& type) const;

		void addNetExecutor(NetExecutor*);

		void killAllNets();
		void triggerCrashDump();

		/**
		 * Returns pointer to virtual rcc device. Is not necessarily available during configure phase of
		 * another device
		 */
		VirtualRCCDevice* getVirtualRCC() const;

		virtual std::list<std::string> getLogEvents() const;

		NetExecutorLookup* getExecutorLookup() const;

		virtual Sync_ID_t createSyncID();
		virtual SynchronizationRuleState getSyncState(Sync_ID_t syncID) const;
		virtual void setSyncState(Sync_ID_t syncID, SynchronizationRuleState newstate);

		virtual unsigned int getNextCPU();
		virtual void releaseCPU(unsigned int cpu);

	protected:
		bool initialize();
		void step();
		void finalize();

	private:
		RegistryImpl();
		virtual ~RegistryImpl();

		bool loadNet(const std::string& desc, const rpi_fragment& fragment, Net_ID_t id, Session_ID_t session, double frequency, bool isRealtime);
		bool parseXML(const std::string& xml, rpi_fragment& out_fragment);
		bool parseDIO(const std::string& dio, rpi_fragment& out_fragment);
		std::map<std::string, std::string> evalFragment(rpi_fragment& fragment);

		RTT::os::Mutex rpiNetCounterMutex;
		mutable RTT::os::Mutex rpiSyncCounterMutex;
		mutable RTT::os::MutexRecursive rpiNetListMutex;
		RTT::os::Mutex executorMutex;
		mutable RTT::os::MutexRecursive devicesMutex;

		Net_ID_t rpiNetCounter;
		Sync_ID_t rpiSyncCounter;
		Session_ID_t sessionCounter;
		std::map<Net_ID_t, std::shared_ptr<Net> > rpiNets;
		std::map<Session_ID_t, std::string> sessions;

		typedef std::pair<Device*, std::string> device_type_t;
		typedef std::map<std::string, device_type_t> devices_t;
		typedef std::multimap<std::string, device_type_t> devices_remove_t;


		devices_t devices;
		devices_remove_t devices_toremove;
		std::map<std::string, std::set<DeviceInstance*> > usedDeviceInstances;

		std::map<Sync_ID_t, SynchronizationRuleState> syncRuleStates;

		std::list<NetExecutor*> executors;

		static RegistryImpl* theRegistry;

		VirtualRCCDevice* vrcc;

		std::list<std::string> logEvents;

		NetExecutorLookup* executorLookup;

		// CPU affinity only on real-time OS
#ifdef HAVE_REALTIME
		const static unsigned int max_cpu = 8;
		unsigned int cpu_load[max_cpu];
#endif
	};


}

#endif /* RPIREGISTRY_H_ */
