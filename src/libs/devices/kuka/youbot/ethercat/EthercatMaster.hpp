/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTETHERCATMASTER_HPP_
#define YOUBOTETHERCATMASTER_HPP_

#include <string>

#include <libs/framework/ethercat/interface/EthercatMaster.hpp>
#include <rcc/DeviceInstanceT.hpp>
#include "EthercatSlave.hpp"

namespace kuka_youbot
{

	class YoubotEthercatMasterNotify
	{
	public:
		virtual ~YoubotEthercatMasterNotify() { }

		virtual void inOp() = 0;
	};

	class YoubotEthercatMaster: YoubotEthercatSlaveNotify
	{
	public:
		YoubotEthercatMaster(YoubotEthercatMasterNotify* parent);
		virtual ~YoubotEthercatMaster();

		bool initialize(const std::string& ethercatDevice, const std::set<std::string>& controllerNames, const std::set<std::string>& auxcontrollerNames);

		int getNumberOfControllerSlaves() const;
		YoubotEthercatSlave* getSlave(int number) const;

		virtual void inSafeOp(int slaveno);
	private:
		RPI::DeviceInstanceT<ethercat::EthercatMaster> ethercatMaster;

		std::vector<YoubotEthercatSlave*> slaves;
		std::vector<DummyEthercatSlave*> auxslaves;

		std::string controllerName;

		YoubotEthercatMasterNotify* parent;
	};

} /* namespace youbot */
#endif /* YOUBOTETHERCATMASTER_HPP_ */
