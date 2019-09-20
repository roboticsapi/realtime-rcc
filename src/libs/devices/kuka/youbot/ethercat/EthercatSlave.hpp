/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTETHERCATSLAVE_HPP_
#define YOUBOTETHERCATSLAVE_HPP_

#include <libs/framework/ethercat/interface/EthercatSlave.hpp>
#include <libs/framework/ethercat/interface/EthercatMaster.hpp>

#include "EthercatSlaveMsg.hpp"
#include "EthercatSlaveMailboxMsg.hpp"

#include <rtt/os/Mutex.hpp>

namespace kuka_youbot
{

	class YoubotEthercatSlaveNotify
	{
	public:
		virtual ~YoubotEthercatSlaveNotify() { }

		virtual void inSafeOp(int slaveno) = 0;
	};

	class YoubotEthercatSlave: public ethercat::EthercatSlave
	{
	public:
		YoubotEthercatSlave(YoubotEthercatSlaveNotify* parent);
		virtual ~YoubotEthercatSlave();

		///stores a mailbox message in a buffer which will be sent to the motor controllers
		///@param msgBuffer ethercat mailbox message
		bool setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer);

		///stores a ethercat message to the buffer
		///@param msgBuffer ethercat message
		void setMsgBuffer(const YouBotSlaveMsg& msgBuffer);

		///get a ethercat message form the buffer
		///@param msgBuffer ethercat message
		YouBotSlaveMsg getMsgBuffer();

		///gets a mailbox message form the buffer which came form the motor controllers
		///@param msgBuffer ethercat mailbox message
		bool getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg);

		void readProcessData(uint8*, uint32);

		void writeProcessData(uint8*, uint32);

		void writeMailboxMessage();
		void readMailboxMessage();

		void inSafeOp();
		bool isinOp() const;
	private:
		RTT::os::Mutex slaveMutex;

		uint8 sendMbx[8], recvMbx[8];
		YouBotSlaveMsg bufMsg;

		bool isOp;
		YoubotEthercatSlaveNotify* parent;
	};

	class DummyEthercatSlave: public ethercat::EthercatSlave
	{
		void readProcessData(uint8*, uint32);
		void writeProcessData(uint8*, uint32);
	};

} /* namespace youbot */
#endif /* YOUBOTETHERCATSLAVE_HPP_ */
