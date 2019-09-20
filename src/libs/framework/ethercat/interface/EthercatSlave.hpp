/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ETHERCATSLAVE_H_
#define ETHERCATSLAVE_H_

#include <string>
#include "EthercatTypes.hpp"

namespace ethercat
{

	enum EthercatMailboxState {
		MBX_STATE_READY, MBX_STATE_WRITE, MBX_STATE_WAIT, MBX_STATE_READ
	};

	class EthercatSlave
	{
	public:
		EthercatSlave();
		virtual ~EthercatSlave();

		/**
		 * Callback function which must fill in data to write to slave
		 */
		virtual void writeProcessData(uint8 *data, uint32 size) = 0;

		/**
		 * Callback function which reads data received from slave
		 */
		virtual void readProcessData(uint8 *data, uint32 size) = 0;

		virtual void inPreOp();
		virtual void inSafeOp();
		//virtual void inOp();

		virtual void failSafeOp();
		//virtual void failOp();

		bool sendMailboxMsg(uint8 *data, uint32 size);
		bool recvMailboxMsg(uint8 *data, uint32 size);

		void setSlaveNo(int num);

		ec_mbxbuft* getMailboxBuffer();
		EthercatMailboxState getMailboxState();
		void setMailboxState(EthercatMailboxState newState);

	protected:

		// TODO: Remove SOEM dependency
		ec_mbxbuft mailboxBuffer;
		EthercatMailboxState mailboxState;

		int slaveno;

	};

} /* namespace ethercat */
#endif /* ETHERCATSLAVE_H_ */
