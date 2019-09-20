/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SOEMETHERCATMASTER_HPP_
#define SOEMETHERCATMASTER_HPP_

#include <string>
#include <list>

#include <rtt/TaskContext.hpp>

#include <rcc/Device.hpp>
#include <rcc/RoundRobinLog.hpp>

#include "../interface/EthercatMaster.hpp"
#include "../interface/EthercatSlave.hpp"

#include "../SOEM/ethercattype.h"
#include <nicdrv.h>
#include "../SOEM/ethercatbase.h"
#include "../SOEM/ethercatmain.h"

namespace ethercat
{

	enum class SOEMState {
		INITIALIZING,
		COLLECTING_SLAVES,
		MAPPING_SLAVES,
		RUNNING,
		RECONFIGURE,
		STOPPING,
		STOPPED
	};

	const double ETHERCAT_CYCLE_TIME = 0.004;// * (1.0-1.0/5800.0);
	const std::string dev_ethercat = "ethercat";

	typedef std::map<int, EthercatSlave*> slavemap;

	class SOEMEthercatMaster: public RTT::TaskContext, public RPI::Device, public EthercatMaster
	{
	public:
		virtual ~SOEMEthercatMaster();

		static SOEMEthercatMaster* createDevice(std::string name, RPI::parameter_t parameters);

		void setEStop(bool);
		void updateParameters();
		std::set<std::string> getMutableParameters() const;

		void addDevice(EthercatDevice* device);
		void removeDevice(EthercatDevice* device);

		std::vector<EthercatSlaveInfo> getSlaveNames() const;
		bool addSlave(int num, EthercatSlave* slave);
		bool removeSlave(EthercatSlave* slave);

		bool doSlaveOp(int slaveno);
		void removeCAflag(int slaveno);

		bool mapSlaves();

		bool readSDO(unsigned int slave, unsigned int index, unsigned int subindex, void* data, int& size);
		bool writeSDO(unsigned int slave, unsigned int index, unsigned int subindex, void* data, int size);

		virtual RTT::os::TimeService::nsecs getNextSync0Time();
	protected:
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();

		void triggerReconfiguration();
		RPI::DeviceState getDeviceState() const;
	private:
		SOEMState state;
		std::list<EthercatDevice*> devices;

		SOEMEthercatMaster(std::string const &name, RPI::parameter_t const &parameters_in);

		std::string ethernetDevice;
		long ethercatTimeout;
		long mailboxTimeout;

		bool initializeEthercat();
		void closeEthercat();

		char IOmap_[4096];

		std::map<int, EthercatSlave*> slaves;
		RTT::os::Mutex slavesMutex;

		int64 realdctime, dcoverruns, lastdc, dcint;
		RTT::os::TimeService::nsecs nextSync0Ticks;

		int lostcnt;

		void updateProcessData();

		double lastCorrection;

		ec_slavet ec_slave[EC_MAXSLAVE];
		int ec_slavecount;
		ec_groupt ec_group[EC_MAXGROUP];
		uint8 esibuf[EC_MAXEEPBUF];
		uint32 esimap[EC_MAXEEPBITMAP];
		ec_eringt ec_elist;
		ec_idxstackT ec_idxstack;
		ec_SMcommtypet ec_SMcommtype;
		ec_PDOassignt ec_PDOassign;
		ec_PDOdesct ec_PDOdesc;
		ec_eepromSMt ec_SM;
		ec_eepromFMMUt ec_FMMU;
		boolean EcatError;
		int64 ec_DCtime;
		ecx_portt ecx_port;
		ecx_redportt ecx_redport;

		ecx_contextt ec_ctx;
		bool ec_init;
		bool request_reconf;

		RPI::RoundRobinLog<double> dumpUpdate;
		RPI::RoundRobinLog<double> dumpWrite;
		RPI::RoundRobinLog<double> dumpSend;
		RPI::RoundRobinLog<double> dumpRecv;
		RPI::RoundRobinLog<double> dumpRead;
		RPI::RoundRobinLog<double> dumpMbx;
		RPI::RoundRobinLog<double> dumpDC;
	};

} /* namespace ethercat */
#endif /* SOEMETHERCATMASTER_HPP_ */
