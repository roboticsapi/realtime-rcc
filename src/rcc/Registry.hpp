/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef RPIREGISTRY_HPP_
#define RPIREGISTRY_HPP_

#include <string>
#include <rtt/os/TimeService.hpp>
#include <list>
#include <memory>

//#include "ExtensionLoader.hpp"
#include "NetFwd.hpp"
#include "DeviceFwd.hpp"
#include "NetModulesFwd.hpp"
#include "NetExecutorFwd.hpp"
#include "DeviceInstanceFwd.hpp"

#ifdef WIN32
const double RPI_DEFAULT_CYCLE_TIME = 0.01;
#else
const double RPI_DEFAULT_CYCLE_TIME = 0.002; // * (1.0 - 0.000003);
#endif

namespace RTT
{
	class TaskContext;
}

namespace RPI
{
	class VirtualRCCDevice;

	typedef int Net_ID_t;

	const Net_ID_t NET_INVALID = -1;

	struct DeviceInfo
	{
		std::string name;
		std::string type;
		std::list<std::string> implementedInterfaces;
	};

	typedef std::vector<DeviceInfo> devicelist_t;

	class RTT_EXPORT Registry
	{
	public:
		static Registry* getRegistry();

		virtual Session_ID_t createSessionID(const std::string& description) = 0;
		virtual std::vector<Session_ID_t> listSessions() = 0;
		virtual std::string getSessionName(Session_ID_t id) = 0;
		virtual std::string getSessionDesc(Session_ID_t id) = 0;
		virtual Session_ID_t getSessionForName(const std::string& name) = 0;
		virtual std::vector<Net_ID_t> listSessionNets(Session_ID_t id) = 0;
		virtual void endSession(Session_ID_t id) = 0;

		virtual Net_ID_t createNetID() = 0;
		virtual NetState getNetState(Net_ID_t id) = 0;
		virtual std::list<NetErrorState> getNetErrorState(Net_ID_t id) = 0;
		virtual std::string getNetName(Net_ID_t id) = 0;
		virtual Net_ID_t getNetForName(const std::string& name) = 0;
		/// get net for name without locking net list (i.e. must only be called inside other registry method)
		virtual Net_ID_t getNetForNameNL(const std::string& name) = 0;
		virtual std::vector<Net_ID_t> listNets() = 0;
		virtual std::map<std::string, std::string> getNetInProperties(Net_ID_t id) = 0;
		virtual Netcomm_Map_t getNetOutProperties(Net_ID_t id) = 0;
		virtual void writeNetInProperties(Net_ID_t id, const std::map<std::string, std::string>& data) = 0;
		virtual std::shared_ptr<NetcommData> getNetcommData(Net_ID_t id, const std::string& key) = 0;

		virtual std::map<std::string, RTT::os::TimeService::nsecs> getNetBlockMinTime(Net_ID_t id) = 0;
		virtual std::map<std::string, RTT::os::TimeService::nsecs> getNetBlockMaxTime(Net_ID_t id) = 0;
		virtual std::map<std::string, RTT::os::TimeService::nsecs> getNetBlockTotalTime(Net_ID_t id) = 0;
		virtual RTT::os::TimeService::nsecs getNetMaxTime(Net_ID_t id) = 0;
		virtual RTT::os::TimeService::nsecs getNetMinTime(Net_ID_t id) = 0;
		virtual RTT::os::TimeService::nsecs getNetMaxCycleTime(Net_ID_t id) = 0;
		virtual RTT::os::TimeService::nsecs getNetMinCycleTime(Net_ID_t id) = 0;

		virtual bool loadNetXML(const std::string& desc, const std::string& xml, Net_ID_t id, Session_ID_t session,
				double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true) = 0;
		virtual bool loadNetDIO(const std::string& desc, const std::string& dio, Net_ID_t id, Session_ID_t session,
				std::string& errors, double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true) = 0;
		virtual std::map<std::string, std::string> evalFragmentXML(std::string xml) = 0;
		virtual std::map<std::string, std::string> evalFragmentDIO(std::string dio) = 0;

		virtual bool startNet(Net_ID_t id) = 0;
		virtual bool scheduleNet(Net_ID_t id, Net_ID_t takeover) = 0;
		virtual void cancelNet(Net_ID_t id) = 0;
		virtual void abortNet(Net_ID_t id) = 0;
		virtual void unloadNet(Net_ID_t id) = 0;
		virtual std::string getNetDescription(Net_ID_t id) = 0;
		virtual std::string getNetXml(Net_ID_t id) = 0;
		virtual std::string getNetDio(Net_ID_t id) = 0;
		virtual std::string getNetDebug(Net_ID_t id) = 0;
		virtual std::string getNetDebug(Net_ID_t id, int& startCycle) = 0;

		virtual bool createDevice(const std::string& type, const std::string& name, const parameter_t& parameters) = 0;
		virtual Device* getDevice(const std::string& name) const = 0;
		virtual Device* getAndAquireDevice(const std::string& name, DeviceInstance* holder) = 0;
		virtual void releaseDevice(const std::string& name, DeviceInstance* holder) = 0;
		virtual void removeDevice(const std::string& name) = 0;
		virtual devicelist_t getDevices() const = 0;
		virtual RPI::DeviceState getDeviceState(std::string name) const = 0;
		virtual DeviceFactory* getDeviceFactory(const std::string& type) const = 0;

		virtual void addNetExecutor(NetExecutor*) = 0;

		virtual void killAllNets() = 0;
		virtual void triggerCrashDump() = 0;
		virtual VirtualRCCDevice* getVirtualRCC() const = 0;

		virtual std::list<std::string> getLogEvents() const = 0;

		virtual unsigned int getNextCPU() = 0;
		virtual void releaseCPU(unsigned int cpu) = 0;
	protected:
		Registry();
		virtual ~Registry();
	};

}

#endif /* RPIREGISTRY_H_ */
