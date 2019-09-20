/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SOEMEthercatMaster.hpp"

#include "../SOEM/ethercattype.h"
#include "../SOEM/ethercatbase.h"
#include "../SOEM/ethercatmain.h"
#include "../SOEM/ethercatconfig.h"
#include "../SOEM/ethercatcoe.h"
#include "../SOEM/ethercatdc.h"
#include "../SOEM/ethercatprint.h"

#include <rtt/Logger.hpp>
#include <rtt/extras/PeriodicActivity.hpp>
#include <rtt/Activity.hpp>
#include <rtt/os/fosi.h>

namespace ethercat
{

	SOEMEthercatMaster::SOEMEthercatMaster(std::string const &name, RPI::parameter_t const &parameters_in) :
			TaskContext("ethercat", PreOperational), Device(name, parameters_in)

	{
		ethernetDevice = getParameter("ethernetdevice", "eth0");
		ethercatTimeout = getParameterT("ethercattimeout", 50);
		mailboxTimeout = getParameterT("mailboxtimeout", 50);

		// initialize IOmap
		memset(IOmap_, 0, sizeof(IOmap_));

		lostcnt = 0;

		lastdc = 0;
		dcoverruns = 0;
		dcint = 0;
		nextSync0Ticks = 0;
		lastCorrection = 1;

		// set up ethercat context
		EcatError = FALSE;
		ec_ctx.slavelist     = &ec_slave[0],
		ec_ctx.port          = &ecx_port,
		ec_ctx.slavecount    = &ec_slavecount,
		ec_ctx.maxslave      = EC_MAXSLAVE,
		ec_ctx.grouplist     = &ec_group[0],
		ec_ctx.maxgroup      = EC_MAXGROUP,
		ec_ctx.esibuf        = &esibuf[0],
		ec_ctx.esimap        = &esimap[0],
		ec_ctx.esislave      = 0,
		ec_ctx.elist         = &ec_elist,
		ec_ctx.idxstack      = &ec_idxstack,
		ec_ctx.ecaterror     = &EcatError,
		ec_ctx.DCtO          = 0,
		ec_ctx.DCl           = 0,
		ec_ctx.DCtime        = &ec_DCtime,
		ec_ctx.SMcommtype    = &ec_SMcommtype,
		ec_ctx.PDOassign     = &ec_PDOassign,
		ec_ctx.PDOdesc       = &ec_PDOdesc,
		ec_ctx.eepSM         = &ec_SM,
		ec_ctx.eepFMMU       = &ec_FMMU;
		ec_init = false;

		dumpUpdate.setName(name + "|UpdateDevice");
		dumpWrite.setName(name + "|WriteProcessData");
		dumpSend.setName(name + "|SendProcessData");
		dumpRecv.setName(name + "|RecvProcessData");
		dumpRead.setName(name + "|ReadProcessData");
		dumpMbx.setName(name + "|Mailbox");
		dumpDC.setName(name + "|Distributed Clock");
		addCrashDumper(&dumpUpdate);
		addCrashDumper(&dumpWrite);
		addCrashDumper(&dumpSend);
		addCrashDumper(&dumpRecv);
		addCrashDumper(&dumpRead);
		addCrashDumper(&dumpMbx);
		addCrashDumper(&dumpDC);

		state = SOEMState::INITIALIZING;
		this->setActivity(
				new RTT::Activity(ORO_SCHED_RT, RTT::os::HighestPriority, 3, 0,
						"EtherCAT Master"));
	}

	void SOEMEthercatMaster::addDevice(EthercatDevice* device)
	{
		devices.push_back(device);
		triggerReconfiguration();
	}

	void SOEMEthercatMaster::removeDevice(EthercatDevice* device)
	{
		devices.remove(device);
	}

	void SOEMEthercatMaster::triggerReconfiguration()
	{
		request_reconf = true;
	}

	bool SOEMEthercatMaster::configureHook()
	{
		return true;
	}

	bool SOEMEthercatMaster::startHook()
	{
		return true;
	}

	void SOEMEthercatMaster::updateHook()
	{
		switch(state) {
		case SOEMState::INITIALIZING:
			this->getActivity()->setPeriod(0);
			RTT::log(RTT::Info) << "Initializing EtherCat." << RTT::endlog();
			if (initializeEthercat()) {
				RTT::log(RTT::Info) << "Initialization completed." << RTT::endlog();
				state = SOEMState::COLLECTING_SLAVES;
			} else {
				RTT::log(RTT::Info) << "Initialization failed." << RTT::endlog();
				stop();
			}
			trigger();
			break;

		case SOEMState::COLLECTING_SLAVES:
			request_reconf = false;
			RTT::log(RTT::Info) << "Collecting slaves from devices." << RTT::endlog();
			for(const auto& device: devices) {
				device->startupDevice();
			}
			RTT::log(RTT::Info) << "Collected " << slaves.size() << " slaves." << RTT::endlog();
			state = SOEMState::MAPPING_SLAVES;
			trigger();
			break;

		case SOEMState::MAPPING_SLAVES:
			RTT::log(RTT::Info) << "Mapping slaves." << RTT::endlog();
			if(mapSlaves()) {
				RTT::log(RTT::Info) << "Slaves mapped." << RTT::endlog();
				this->getActivity()->setPeriod(ETHERCAT_CYCLE_TIME);
				state = SOEMState::RUNNING;
			} else {
				RTT::log(RTT::Info) << "No slaves mapped." << RTT::endlog();
				this->getActivity()->setPeriod(1);
				state = SOEMState::RUNNING;
			}
			break;

		case SOEMState::RUNNING:
			updateProcessData();
			if(request_reconf)
				state = SOEMState::RECONFIGURE;
			break;

		case SOEMState::RECONFIGURE:
			updateProcessData();
			RTT::log(RTT::Info) << "EtherCat bus reconfiguration" << RTT::endlog();
			for(const auto& device: devices) {
				device->shutdownDevice();
			}
			state = SOEMState::STOPPING;
			break;

		case SOEMState::STOPPING:
			updateProcessData();
			if(slaves.size() == 0) {
				state = SOEMState::STOPPED;
			}
			break;

		case SOEMState::STOPPED:
			RTT::log(RTT::Info) << "EtherCat bus stopped" << RTT::endlog();
			this->getActivity()->setPeriod(0.5);
			closeEthercat();
			state = SOEMState::INITIALIZING;
			trigger();
			break;
		}
	}

	SOEMEthercatMaster::~SOEMEthercatMaster()
	{
		this->stop();

		// acquire lock to prevent destructor from completing while update hook is still running
		RTT::os::MutexLock lock(slavesMutex);
		closeEthercat();
	}

	SOEMEthercatMaster* SOEMEthercatMaster::createDevice(std::string name, RPI::parameter_t parameters)
	{
		SOEMEthercatMaster* ret = new SOEMEthercatMaster(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	void SOEMEthercatMaster::setEStop(bool)
	{
		// do nothing, ethercat bus cannot be stopped
	}

	void SOEMEthercatMaster::updateParameters()
	{
		// TODO: Update parameters (if possible)
	}

	std::set<std::string> SOEMEthercatMaster::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	bool SOEMEthercatMaster::initializeEthercat()
	{
		/* initialise SOEM, bind socket to ifname */
		if (ecx_init(&ec_ctx, (char*)ethernetDevice.c_str()))
		{
			RTT::log(RTT::Info) << "Initializing EtherCAT on " << ethernetDevice << RTT::endlog();

			/* find and auto-config slaves */
			ecx_config_init(&ec_ctx, TRUE);

		} else
		{
			RTT::log(RTT::Critical) << " No socket connection. Run as root!" << RTT::endlog();
			//throw std::runtime_error("No socket connection on " + ethernetDevice + "\nExcecute as root");
			return false;
		}

		ec_init = true;

//		for (int cnt = 1; cnt <= ec_slavecount; cnt++)
//		 {
//		 RTT::log(RTT::Info) << "Slave: " << cnt << " Name: " << ec_slave[cnt].name << " Output size: "
//		 << ec_slave[cnt].Obits << "bits Input size: " << ec_slave[cnt].Ibits << "bits State: "
//		 << ec_slave[cnt].state << " delay: " << ec_slave[cnt].pdelay << RTT::endlog(); //<< " has dclock: " << (bool)ec_slave[cnt].hasdc;
//
//		 }

		for (int cnt = 1; cnt <= *ec_ctx.slavecount; cnt++)
		{
			RTT::log(RTT::Info) << "Slave: " << cnt << " Name: " << ec_ctx.slavelist[cnt].name <<
					" Manufacturer: " << ec_ctx.slavelist[cnt].eep_man <<
					" ID: " << ec_ctx.slavelist[cnt].eep_id <<
					RTT::endlog();

		}

		return true;
	}

	bool SOEMEthercatMaster::mapSlaves()
	{
		{
			RTT::os::MutexLock lock(slavesMutex);

			// notify all slaves that we are in PreOp
			for (slavemap::const_iterator it = slaves.begin(); it != slaves.end(); ++it)
			{
				it->second->inPreOp();
			}
		}

		ecx_configdc(&ec_ctx);

		ecx_statecheck(&ec_ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

		// Sleep 100ms to allow EtherCAT devices to settle
		usleep(100000);

		if (ecx_config_map_group(&ec_ctx, &IOmap_, 0) > 0)
		{
			//bool dc = ec_configdc();

			RTT::log(RTT::Debug) << *ec_ctx.slavecount << " slaves found and configured." << RTT::endlog();

			/* wait for all slaves to reach SAFE_OP state */
			ecx_statecheck(&ec_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
			if (ec_ctx.slavelist[0].state != EC_STATE_SAFE_OP)
			{
				RTT::log(RTT::Warning) << "Not all slaves reached safe operational state." << RTT::endlog();
			}
			ecx_dcsync0(&ec_ctx, 1, true, ETHERCAT_CYCLE_TIME * 1e9, 100000000);

			ecx_readstate(&ec_ctx);

			ecx_send_processdata(&ec_ctx);
			ecx_receive_processdata(&ec_ctx, EC_TIMEOUTRET);

			// Notify all slaves of state
			slavemap::iterator slit;
			for (int i = 1; i <= *ec_ctx.slavecount; i++)
			{
				slit = slaves.find(i);
				if (ec_ctx.slavelist[i].state != EC_STATE_SAFE_OP)
				{
					RTT::log(RTT::Info) << "Slave " << i << " State=" << ec_ctx.slavelist[i].state << " StatusCode="
							<< ec_ctx.slavelist[i].ALstatuscode << " : " << ec_ALstatuscode2string(ec_ctx.slavelist[i].ALstatuscode)
							<< RTT::endlog();

					if (slit != slaves.end())
						slit->second->failSafeOp();
				} else
				{
					if (slit != slaves.end())
						slit->second->inSafeOp();

				}
			}

		} else
		{
			RTT::log(RTT::Critical) << " No EtherCAT slaves found." << RTT::endlog();

			return false;
		}

		return true;
	}

	void SOEMEthercatMaster::closeEthercat()
	{
		if(!ec_init) return;
		ec_ctx.slavelist[0].state = EC_STATE_INIT;

		/* request SAFE_OP state for all slaves */
		ecx_writestate(&ec_ctx, 0);

		//stop SOEM, close socket
		ecx_close(&ec_ctx);
	}

	std::vector<EthercatSlaveInfo> SOEMEthercatMaster::getSlaveNames() const
	{
		std::vector<EthercatSlaveInfo> ret;

		for (int cnt = 1; cnt <= *ec_ctx.slavecount; cnt++)
		{
			EthercatSlaveInfo esi(cnt, ec_ctx.slavelist[cnt].name, ec_ctx.slavelist[cnt].eep_man, ec_ctx.slavelist[cnt].eep_id,
					ec_ctx.slavelist[cnt].eep_rev);
			ret.push_back(esi);
		}

		return ret;
	}

	bool SOEMEthercatMaster::addSlave(int num, EthercatSlave* slave)
	{
		if(state != SOEMState::COLLECTING_SLAVES)
			return false;

		RTT::os::MutexLock lock(slavesMutex);

		// don't overwrite existing slaves
		if (slaves.find(num) != slaves.end())
			return false;

		slaves[num] = slave;

		slave->setSlaveNo(num);
		return true;
	}

	bool SOEMEthercatMaster::removeSlave(EthercatSlave* slave)
	{
		RTT::os::MutexLock lock(slavesMutex);

		bool found = false;
		// find slave in list
		for (std::map<int, EthercatSlave*>::iterator it = slaves.begin(); it != slaves.end();)
		{
			if (it->second == slave)
			{
				slaves.erase(it++);
				found = true;
			} else
			{
				++it;
			}
		}

		return found;
	}

	void SOEMEthercatMaster::updateProcessData()
	{
		RTT::os::MutexLock lock(slavesMutex);

		RTT::os::TimeService::nsecs begin = RTT::os::TimeService::Instance()->getNSecs();
		for(const auto& device: devices) {
			device->updateDevice();
		}
		RTT::os::TimeService::nsecs afterupdate = RTT::os::TimeService::Instance()->getNSecs();

		std::map<int, EthercatSlave*>::iterator slaveit;

		for (const auto& slave: slaves)
		{
			slave.second->writeProcessData(ec_ctx.slavelist[slave.first].outputs, ec_ctx.slavelist[slave.first].Obytes);
		}

		RTT::os::TimeService::nsecs afterwrite = RTT::os::TimeService::Instance()->getNSecs();
		// Send processdata, uninitialise if connection has been interrupted
		ecx_send_processdata(&ec_ctx);

		RTT::os::TimeService::nsecs aftersend = RTT::os::TimeService::Instance()->getNSecs();

#ifdef REALTIME
		TIME_SPEC sleepts;
		sleepts.tv_nsec = 200 * 1e3;
		sleepts.tv_sec = 0;
		rtos_nanosleep(&sleepts, 0);
#else
		usleep(200);
#endif

		RTT::os::TimeService::nsecs aftersleep = RTT::os::TimeService::Instance()->getNSecs();
		// Receive data from slaves
		int rcv = ecx_receive_processdata(&ec_ctx, this->ethercatTimeout);

		if(rcv == 0)  {
			lostcnt++;
		} else {
			lostcnt = 0;
		}

		RTT::os::TimeService::nsecs afterreceive = RTT::os::TimeService::Instance()->getNSecs();

//		long delta = RTT::os::TimeService::Instance()->getNSecs(begin);
//		if (delta > 150000)
//		{
//			std::cout << "Receive slow " << delta << std::endl;
//		}

		if (*ec_ctx.DCtime < lastdc)
			dcoverruns += 4294967296;
		lastdc = *ec_ctx.DCtime;

		int64 dcCycle = (*ec_ctx.DCtime + dcoverruns) - realdctime;
		realdctime = *ec_ctx.DCtime + dcoverruns;

		int64 dclate = ETHERCAT_CYCLE_TIME * 1e9 / 2;

		int64 dcdelta = (realdctime - dclate) % (int64) (ETHERCAT_CYCLE_TIME * 1e9);
		if (dcdelta > ETHERCAT_CYCLE_TIME * 1e9 / 2)
			dcdelta -= ETHERCAT_CYCLE_TIME * 1e9;

		if (dcdelta > 0)
			dcint++;
		if (dcdelta < 0)
			dcint--;

		double correction = (1 - 1.0*dcdelta / (double) (ETHERCAT_CYCLE_TIME * 1e9) - dcint / (double) 5000);

		double alpha = 0.2;
		correction = lastCorrection * (1-alpha) + correction * alpha;
		lastCorrection = correction;
		setPeriod(ETHERCAT_CYCLE_TIME * correction);// - delta*1e-9);

		nextSync0Ticks = aftersend//RTT::os::TimeService::Instance()->getTicks()
				- ((dclate + ETHERCAT_CYCLE_TIME * 1e9) * correction);

		uint16 w = 0x0000;
	    int wkc = ecx_BRD(ec_ctx.port, 0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);		/* detect number of slaves */
		if(ec_slavecount != wkc && (ec_slavecount != 0 || wkc != -1)) {
			RTT::log(RTT::Info) << "WKC " << wkc << ", SC " << ec_slavecount << RTT::endlog();
			triggerReconfiguration();
		}

		for (const auto& slave: slaves)
		{
			slave.second->readProcessData(ec_ctx.slavelist[slave.first].inputs, ec_ctx.slavelist[slave.first].Ibytes);
		}
		RTT::os::TimeService::nsecs afterread = RTT::os::TimeService::Instance()->getNSecs();

		// Mailbox messages
		for (const auto& slave: slaves)
		{
			if(slave.second->getMailboxState() == MBX_STATE_WRITE) {
				if(ecx_mbxsend(&ec_ctx, slave.first, slave.second->getMailboxBuffer(), mailboxTimeout)) {
					slave.second->setMailboxState(MBX_STATE_WAIT);
				}
			}
			//receive mailbox messages to buffer
			if (slave.second->getMailboxState() == MBX_STATE_WAIT)
			{
				if (ecx_mbxreceive(&ec_ctx, slave.first, slave.second->getMailboxBuffer(), mailboxTimeout)) {
					slave.second->setMailboxState(MBX_STATE_READ);
				}
			}
		}
		RTT::os::TimeService::nsecs aftermbx = RTT::os::TimeService::Instance()->getNSecs();

		dumpUpdate.put((afterupdate-begin)/1e9, aftersend);
		dumpWrite.put((afterwrite-afterupdate)/1e9, aftersend);
		dumpSend.put((aftersend-afterwrite)/1e9, aftersend);
		dumpRecv.put((afterreceive-aftersleep)/1e9, aftersend);
		dumpRead.put((afterread-afterreceive)/1e9, aftersend);
		dumpMbx.put((aftermbx-afterread)/1e9, aftersend);
		dumpDC.put(dcCycle/1e9, aftersend);
	}

	RPI::DeviceState SOEMEthercatMaster::getDeviceState() const
	{
		return state == SOEMState::RUNNING ? RPI::DeviceState::OPERATIONAL : RPI::DeviceState::OFFLINE;
	}

	bool SOEMEthercatMaster::doSlaveOp(int slaveno)
	{
		//RTT::log(RTT::Debug) << "Request operational state for all slaves" << RTT::endlog();

		ec_ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
		// request OP state for all slaves
		/* send one valid process data to make outputs in slaves happy*/
		/* request OP state for all slaves */
		ecx_writestate(&ec_ctx, 0);
		// wait for all slaves to reach OP state

		ecx_statecheck(&ec_ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
		if (ec_ctx.slavelist[0].state == EC_STATE_OPERATIONAL)
		{
			//RTT::log(RTT::Debug) << "Operational state reached for all slaves." << RTT::endlog();
			return true;
		} else
		{
			//RTT::log(RTT::Critical) << " Not all slaves reached operational state." << RTT::endlog();
			//throw std::runtime_error("Not all slaves reached operational state.");
			return false;

		}

	}

	void SOEMEthercatMaster::removeCAflag(int slaveno)
	{
		if (slaveno > 0)
			ec_ctx.slavelist[slaveno].CoEdetails &= ~ECT_COEDET_SDOCA;
	}

	bool SOEMEthercatMaster::readSDO(unsigned int slave, unsigned int index, unsigned int subindex, void* data,
			int& size)
	{
		return ecx_SDOread(&ec_ctx, slave, index, subindex, false, &size, data, EC_TIMEOUTRXM);
	}

	bool SOEMEthercatMaster::writeSDO(unsigned int slave, unsigned int index, unsigned int subindex, void* data,
			int size)
	{
		return ecx_SDOwrite(&ec_ctx, slave, index, subindex, false, size, data, EC_TIMEOUTRXM);
	}

	RTT::os::TimeService::nsecs SOEMEthercatMaster::getNextSync0Time()
	{
		return nextSync0Ticks;
	}

} /* namespace ethercat */
