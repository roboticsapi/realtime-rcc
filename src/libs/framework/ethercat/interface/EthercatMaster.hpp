/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ETHERCATMASTER_HPP_
#define ETHERCATMASTER_HPP_

#include <rtt/os/TimeService.hpp>
#include "EthercatSlave.hpp"
#include <vector>
#include "EthercatDevice.hpp"

namespace ethercat {

	class EthercatSlaveInfo
	{
	public:
		unsigned int slaveID;
		std::string slaveName;
		unsigned int manufacturer;
		unsigned int ID;
		unsigned int revision;

		EthercatSlaveInfo(unsigned int slaveID, std::string slaveName, unsigned int manufacturer, unsigned int ID,
				unsigned int revision)
		{
			this->slaveID = slaveID;
			this->slaveName = slaveName;
			this->manufacturer = manufacturer;
			this->ID = ID;
			this->revision = revision;
		}

		EthercatSlaveInfo() :
				slaveID(-1), slaveName(""), manufacturer(0), ID(0), revision(0)
		{
		}
	};

	class EthercatMaster
	{
	public:
		virtual ~EthercatMaster() { };

		virtual void addDevice(EthercatDevice* device) = 0;
		virtual void removeDevice(EthercatDevice* device) = 0;

		virtual std::vector<EthercatSlaveInfo> getSlaveNames() const = 0;
		virtual bool addSlave(int num, EthercatSlave* slave) = 0;
		virtual bool removeSlave(EthercatSlave* slave) = 0;

		virtual bool doSlaveOp(int slaveno) = 0;
		virtual void removeCAflag(int slaveno) = 0;

		virtual bool mapSlaves() = 0;

		virtual bool readSDO(unsigned int slave, unsigned int index, unsigned int subindex, void* data, int& size) = 0;
		virtual bool writeSDO(unsigned int slave, unsigned int index, unsigned int subindex, void* data, int size) = 0;

		virtual RTT::os::TimeService::nsecs getNextSync0Time() = 0;
	};

}
#endif /* ETHERCATMASTER_HPP_ */
