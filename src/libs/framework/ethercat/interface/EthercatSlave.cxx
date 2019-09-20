/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EthercatSlave.hpp"
#include "string.h"
#include <iostream>

#include "../SOEM/ethercattype.h"
#include <nicdrv.h>
#include "../SOEM/ethercatbase.h"
#include "../SOEM/ethercatmain.h"

namespace ethercat
{

	EthercatSlave::EthercatSlave() :
			slaveno(0), mailboxState(MBX_STATE_READY)
	{
		ec_clearmbx(&mailboxBuffer);
	}

	EthercatSlave::~EthercatSlave()
	{

	}

	void EthercatSlave::inPreOp()
	{

	}

	void EthercatSlave::inSafeOp()
	{

	}

	void EthercatSlave::failSafeOp()
	{

	}

	bool EthercatSlave::sendMailboxMsg(uint8 *data, uint32 size)
	{
		if(mailboxState != MBX_STATE_READY) return false;
		memcpy(mailboxBuffer, data, size);

//		std::cout << "Sent to slave " << slaveno << ": " << std::hex;
//		for(int i=0; i<size; i++)
//			std::cout << (int)data[i] << " ";
//		std::cout << std::dec << "." << std::endl;

		mailboxState = MBX_STATE_WRITE;
		return true;
	}

	bool EthercatSlave::recvMailboxMsg(uint8 *data, uint32 size)
	{
		if(mailboxState != MBX_STATE_READ) return false;
		memcpy(data, mailboxBuffer, size);

//		std::cout << "Received from slave " << slaveno << ": " << std::hex;
//		for(int i=0; i<size; i++)
//			std::cout << (int)data[i] << " ";
//		std::cout << std::dec << "." << std::endl;

		mailboxState = MBX_STATE_READY;
		return true;
	}

	EthercatMailboxState EthercatSlave::getMailboxState() {
		return mailboxState;
	}

	void EthercatSlave::setMailboxState(EthercatMailboxState newState) {
		mailboxState = newState;
	}

	ec_mbxbuft* EthercatSlave::getMailboxBuffer()
	{
		return &mailboxBuffer;
	}

	void EthercatSlave::setSlaveNo(int num)
	{
		slaveno = num;
	}

} /* namespace ethercat */
