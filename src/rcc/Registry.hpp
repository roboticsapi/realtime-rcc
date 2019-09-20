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
#include <set>
#include <memory>

#include "NetFwd.hpp"
#include "DeviceFwd.hpp"
#include "NetModulesFwd.hpp"
#include "NetExecutorFwd.hpp"
#include "DeviceInstanceFwd.hpp"
#include "NetExecutorLookupFwd.hpp"
#include "scheduling/SchedulingCondition.hpp"
#include "SynchronizationRuleFwd.hpp"

#ifdef WIN32
const double RPI_DEFAULT_CYCLE_TIME = 0.01;
#else
const double RPI_DEFAULT_CYCLE_TIME = 0.003; // * (1.0 - 0.000003);
#endif

namespace RTT
{
	class TaskContext;
}

namespace RPI
{
	class VirtualRCCDevice;

	struct DeviceInfo
	{
		std::string name;
		std::string type;
		std::map<std::string,std::map<std::string,std::string>> implementedInterfaces;
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
		virtual Net_ID_t getNetID(const std::string& name) const = 0;
		virtual std::shared_ptr<Net> getNet(Net_ID_t netID) const = 0;
		virtual std::shared_ptr<Net> getNet(const std::string& name) const = 0;

		virtual std::vector<Net_ID_t> listNets() = 0;

		virtual bool loadNetXML(const std::string& desc, const std::string& xml, Net_ID_t id, Session_ID_t session,
				double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true) = 0;
		virtual bool loadNetDIO(const std::string& desc, const std::string& dio, Net_ID_t id, Session_ID_t session,
				std::string& errors, double frequency = RPI_DEFAULT_CYCLE_TIME, bool isRealtime = true) = 0;
		virtual std::map<std::string, std::string> evalFragmentXML(std::string xml) = 0;
		virtual std::map<std::string, std::string> evalFragmentDIO(std::string dio) = 0;

		virtual bool createDevice(const std::string& type, const std::string& name, const parameter_t& parameters) = 0;
		virtual Device* getDevice(const std::string& name) const = 0;
		virtual Device* getAndAquireDevice(const std::string& name, DeviceInstance* holder) = 0;
		virtual void releaseDevice(const std::string& name, DeviceInstance* holder) = 0;
		virtual void removeDevice(const std::string& name) = 0;
		virtual devicelist_t getDevices() const = 0;
		virtual RPI::DeviceState getDeviceState(std::string name) const = 0;
		virtual RTT::os::TimeService::nsecs getDeviceLastUpdated(std::string name) const = 0;
		virtual DeviceFactory* getDeviceFactory(const std::string& type) const = 0;

		virtual void addNetExecutor(NetExecutor*) = 0;

		virtual void killAllNets() = 0;
		virtual void triggerCrashDump() = 0;
		virtual VirtualRCCDevice* getVirtualRCC() const = 0;

		virtual std::list<std::string> getLogEvents() const = 0;

		virtual NetExecutorLookup* getExecutorLookup() const = 0;

		virtual Sync_ID_t createSyncID() = 0;
		virtual SynchronizationRuleState getSyncState(Sync_ID_t) const = 0;
		virtual void setSyncState(Sync_ID_t syncID, SynchronizationRuleState newstate) = 0;

		virtual unsigned int getNextCPU() = 0;
		virtual void releaseCPU(unsigned int cpu) = 0;
	protected:
		Registry();
		virtual ~Registry();
	};

}

#endif /* RPIREGISTRY_H_ */
