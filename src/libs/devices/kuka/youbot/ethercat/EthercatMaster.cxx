/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EthercatMaster.hpp"

namespace kuka_youbot
{

	YoubotEthercatMaster::YoubotEthercatMaster(YoubotEthercatMasterNotify* parent) : parent(parent)
	{

	}

	YoubotEthercatMaster::~YoubotEthercatMaster()
	{
		ethercat::EthercatMaster* master = ethercatMaster.getDevice();

		if (master)
		{

			for (std::vector<YoubotEthercatSlave*>::iterator it = slaves.begin(); it != slaves.end(); ++it)
			{
				YoubotEthercatSlave* slave = *it;

				master->removeSlave(slave);

				delete slave;
			}

			for (std::vector<DummyEthercatSlave*>::iterator it = auxslaves.begin(); it != auxslaves.end(); ++it)
			{
				DummyEthercatSlave* slave = *it;

				master->removeSlave(slave);

				delete slave;
			}
		}
		slaves.clear();
		auxslaves.clear();

	}

	bool YoubotEthercatMaster::initialize(const std::string& ethercatDevice,
			const std::set<std::string>& controllerNames, const std::set<std::string>& auxControllerNames)
	{
		bool found = false;
		ethercatMaster.fetchInstance(ethercatDevice);

		ethercat::EthercatMaster* master = ethercatMaster.getDevice();
		if (master)
		{
			std::vector<ethercat::EthercatSlaveInfo> slaveNames = master->getSlaveNames();

			for (unsigned int i = 0; i < slaveNames.size(); i++)
			{
				if (controllerNames.find(slaveNames[i].slaveName) != controllerNames.end())
				{
					YoubotEthercatSlave* slave = new YoubotEthercatSlave(this);
					this->slaves.push_back(slave);
					ethercatMaster.getDevice()->addSlave(slaveNames[i].slaveID, slave);
					found = true;
				}

				// Auxiliary slaves are also added
				if (auxControllerNames.find(slaveNames[i].slaveName) != auxControllerNames.end())
				{
					DummyEthercatSlave* slave = new DummyEthercatSlave();
					this->auxslaves.push_back(slave);
					ethercatMaster.getDevice()->addSlave(slaveNames[i].slaveID, slave);
				}
			}

		}
		return found;
	}

	YoubotEthercatSlave* YoubotEthercatMaster::getSlave(int number) const
	{
		if (number >= 0 && number < slaves.size())
			return slaves[number];
		else
			return 0;
	}

	int YoubotEthercatMaster::getNumberOfControllerSlaves() const
	{
		return slaves.size();
	}

	void YoubotEthercatMaster::inSafeOp(int slaveno)
	{
		ethercatMaster.getDevice()->doSlaveOp(slaveno);

		bool allop = true;

		for (std::vector<YoubotEthercatSlave*>::iterator it = slaves.begin(); it != slaves.end(); ++it)
		{
			allop &= (*it)->isinOp();
		}

		if(allop)
			parent->inOp();
	}

} /* namespace youbot */
