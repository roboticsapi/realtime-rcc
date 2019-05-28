/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef RPIREGISTRYIMPL_HPP_
#define RPIREGISTRYIMPL_HPP_

#include <string>
#include <rtt/os/TimeService.hpp>
#include <rtt/TaskContext.hpp>

#include "ExtensionLoader.hpp"
#include "Net.hpp"
#include "Device.hpp"
#include "Registry.hpp"


namespace RPI
{
	class VirtualRCCDevice;

	typedef int Net_ID_t;

	class RTT_EXPORT RegistryImpl: public Registry, public RTT::TaskContext
	{
	public:
		static RegistryImpl* getRegistry();
		static void clean();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		Session_ID_t createSessionID(const std::string& description);
		std::vector<Session_ID_t> listSessions();
		std::string getSessionName(Session_ID_t id);
		std::string getSessionDesc(Session_ID_t id);
		Session_ID_t getSessionForName(const std::string& name);
		std::vector<Net_ID_t> listSessionNets(Session_ID_t id);
		void endSession(Session_ID_t id);

		Net_ID_t createNetID();
		NetState getNetState(Net_ID_t id);
		std::list<NetErrorState> getNetErrorState(Net_ID_t id);
		std::string getNetName(Net_ID_t id);
		Net_ID_t getNetForName(const std::string& name);
		Net_ID_t getNetForNameNL(const std::string& name);
		std::vector<Net_ID_t> listNets();
		std::map<std::string, std::string> getNetInProperties(Net_ID_t id);
		Netcomm_Map_t getNetOutProperties(Net_ID_t id);
		void writeNetInProperties(Net_ID_t id, const std::map<std::string, std::string>& data);
		virtual std::shared_ptr<NetcommData> getNetcommData(Net_ID_t id, const std::string& key);

		std::map<std::string,RTT::os::TimeService::nsecs> getNetBlockMinTime(Net_ID_t id);
		std::map<std::string,RTT::os::TimeService::nsecs> getNetBlockMaxTime(Net_ID_t id);
		std::map<std::string,RTT::os::TimeService::nsecs> getNetBlockTotalTime(Net_ID_t id);
		RTT::os::TimeService::nsecs getNetMaxTime(Net_ID_t id);
		RTT::os::TimeService::nsecs getNetMinTime(Net_ID_t id);
		RTT::os::TimeService::nsecs getNetMaxCycleTime(Net_ID_t id);
		RTT::os::TimeService::nsecs getNetMinCycleTime(Net_ID_t id);

		bool loadNetXML(const std::string& desc, const std::string& xml, Net_ID_t id, Session_ID_t session, double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true);
		bool loadNetDIO(const std::string& desc, const std::string& dio, Net_ID_t id, Session_ID_t session, std::string& errors, double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true);
		std::map<std::string, std::string> evalFragmentXML(const std::string xml);
		std::map<std::string, std::string> evalFragmentDIO(const std::string dio);

		bool startNet(Net_ID_t id);
		bool scheduleNet(Net_ID_t id, Net_ID_t takeover);
		void cancelNet(Net_ID_t id);
		void abortNet(Net_ID_t id);
		void unloadNet(Net_ID_t id);
		std::string getNetDescription(Net_ID_t id);
		std::string getNetXml(Net_ID_t id);
		std::string getNetDio(Net_ID_t id);
		std::string getNetDebug(Net_ID_t id);
		std::string getNetDebug(Net_ID_t id, int& startCycle);

		bool createDevice(const std::string& type, const std::string& name, const parameter_t& parameters);
		Device* getDevice(const std::string& name) const;
		Device* getAndAquireDevice(const std::string& name, DeviceInstance* holder);
		void releaseDevice(const std::string& name, DeviceInstance* holder);
		void removeDevice(const std::string& name);
		devicelist_t getDevices() const;
		virtual RPI::DeviceState getDeviceState(std::string name) const;
		DeviceFactory* getDeviceFactory(const std::string& type) const;

		void addNetExecutor(NetExecutor*);

		void killAllNets();
		void triggerCrashDump();
		//RTT::Method<void(void)> killAllNetsMethod;

		/**
		 * Returns pointer to virtual rcc device. Is not necessarily available during configure phase of
		 * another device
		 */
		VirtualRCCDevice* getVirtualRCC() const;

		virtual std::list<std::string> getLogEvents() const;

		virtual unsigned int getNextCPU();
		virtual void releaseCPU(unsigned int cpu);
	private:
		RegistryImpl();
		virtual ~RegistryImpl();

		bool loadNet(const std::string& desc, const rpi_fragment& fragment, Net_ID_t id, Session_ID_t session, double frequency, bool isRealtime);
		bool parseXML(const std::string& xml, rpi_fragment& out_fragment);
		bool parseDIO(const std::string& dio, rpi_fragment& out_fragment);
		std::map<std::string, std::string> evalFragment(rpi_fragment& fragment);

		RTT::os::Mutex rpiNetCounterMutex;
		RTT::os::Mutex rpiNetListMutex;
		RTT::os::Mutex executorMutex;
		mutable RTT::os::MutexRecursive devicesMutex;

		Net_ID_t rpiNetCounter;
		Session_ID_t sessionCounter;
		std::map<Net_ID_t, Net*> rpiNets;
		std::map<Session_ID_t, std::string> sessions;

		typedef std::pair<Device*, std::string> device_type_t;
		typedef std::map<std::string, device_type_t> devices_t;
		typedef std::multimap<std::string, device_type_t> devices_remove_t;


		devices_t devices;
		devices_remove_t devices_toremove;
		std::map<std::string, std::set<DeviceInstance*> > usedDeviceInstances;

		std::list<NetExecutor*> executors;

		static RegistryImpl* theRegistry;

		Net* getNet(Net_ID_t netID) const;

		VirtualRCCDevice* vrcc;

		std::list<std::string> logEvents;

		// CPU affinity only on real-time OS
#ifdef HAVE_REALTIME
		const static unsigned int max_cpu = 8;
		unsigned int cpu_load[max_cpu];
#endif
	};

}

#endif /* RPIREGISTRY_H_ */
