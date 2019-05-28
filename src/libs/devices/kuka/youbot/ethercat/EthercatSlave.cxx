/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EthercatSlave.hpp"

#include <rtt/os/MutexLock.hpp>
#include <rtt/Logger.hpp>

namespace kuka_youbot
{

	YoubotEthercatSlave::YoubotEthercatSlave(YoubotEthercatSlaveNotify* parent) :
			slaveMutex(), parent(parent)
	{
		isOp = false;
	}

	YoubotEthercatSlave::~YoubotEthercatSlave()
	{
	}

	///stores a ethercat message to the buffer
	///@param msgBuffer ethercat message
	///@param jointNumber joint number of the sender joint
	void YoubotEthercatSlave::setMsgBuffer(const YouBotSlaveMsg& msgBuffer)
	{
		RTT::os::MutexLock lock(slaveMutex);
		bufMsg.stctOutput = msgBuffer.stctOutput;

	}

	///get a ethercat message form the buffer
	///@param msgBuffer ethercat message
	///@param jointNumber joint number of the receiver joint
	YouBotSlaveMsg YoubotEthercatSlave::getMsgBuffer()
	{
		RTT::os::MutexLock lock(slaveMutex);
		return bufMsg;
	}

	///stores a mailbox message in a buffer which will be sent to the motor controllers
	///@param msgBuffer ethercat mailbox message
	///@param jointNumber joint number of the sender joint
	bool YoubotEthercatSlave::setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer)
	{
		RTT::os::MutexLock lock(slaveMutex);

		sendMbx[0] = msgBuffer.stctOutput.moduleAddress;
		sendMbx[1] = msgBuffer.stctOutput.commandNumber;
		sendMbx[2] = msgBuffer.stctOutput.typeNumber;
		sendMbx[3] = msgBuffer.stctOutput.motorNumber;
		sendMbx[4] = msgBuffer.stctOutput.value >> 24;
		sendMbx[5] = msgBuffer.stctOutput.value >> 16;
		sendMbx[6] = msgBuffer.stctOutput.value >> 8;
		sendMbx[7] = msgBuffer.stctOutput.value & 0xff;
		return EthercatSlave::sendMailboxMsg(sendMbx, 8);
	}

	///gets a mailbox message form the buffer which came form the motor controllers
	///@param msgBuffer ethercat mailbox message
	///@param jointNumber joint number of the receiver joint
	bool YoubotEthercatSlave::getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg)
	{
		RTT::os::MutexLock lock(slaveMutex);

		if(!EthercatSlave::recvMailboxMsg(recvMbx, 8))
			return false;

		mailboxMsg.stctInput.replyAddress = (int) recvMbx[0];
		mailboxMsg.stctInput.moduleAddress = (int) recvMbx[1];
		mailboxMsg.stctInput.status = (int) recvMbx[2];
		mailboxMsg.stctInput.commandNumber = (int) recvMbx[3];
		mailboxMsg.stctInput.value = (recvMbx[4] << 24 | recvMbx[5] << 16
						| recvMbx[6] << 8 | recvMbx[7]);
		return true;
	}

	void YoubotEthercatSlave::readProcessData(uint8* data, uint32 size)
	{
		RTT::os::MutexLock lock(slaveMutex);

		// data must be big enough to being cast to SlaveMessageInput
		if (size >= sizeof(bufMsg.stctInput))
		{
			bufMsg.stctInput = *((SlaveMessageInput*) data);
		} else
		{
			RTT::log(RTT::Error) << "Input size does not match" << RTT::endlog();
		}
	}

	void YoubotEthercatSlave::writeProcessData(uint8* data, uint32 size)
	{
		RTT::os::MutexLock lock(slaveMutex);

		// data size must be big enough to read
		if (size >= sizeof(bufMsg.stctOutput))
		{
			*((SlaveMessageOutput*) data) = bufMsg.stctOutput;
		} else
		{
			RTT::log(RTT::Error) << "Output size does not match" << RTT::endlog();
		}
	}

	void YoubotEthercatSlave::inSafeOp()
	{
		isOp = true;
		parent->inSafeOp(slaveno);
	}

	bool YoubotEthercatSlave::isinOp() const
	{
		return isOp;
	}

	void DummyEthercatSlave::readProcessData(uint8*, uint32)
	{

	}
	void DummyEthercatSlave::writeProcessData(uint8*, uint32)
	{

	}

} /* namespace youbot */
