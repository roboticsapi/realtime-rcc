/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANOPENMASTERDEVICE_HPP_
#define CANOPENMASTERDEVICE_HPP_

namespace canopen {
	typedef unsigned char  uint8;

	enum NodeMode {
		NM_OPERATIONAL = 0x01, NM_STOP = 0x02, NM_PRE_OPERATIONAL = 0x80,
		NM_RESET_APPLICATION = 0x81, NM_RESET_COMMUNICATION = 0x82
	};
	enum NodeState {
		NS_BOOTUP = 0x01, NS_STOPPED = 0x04, NS_OPERATIONAL = 0x05, NS_PRE_OPERATIONAL = 0x7F
	};

	class NodeStateListener
	{
	public:
		NodeStateListener() {};
		virtual ~NodeStateListener() {};
		virtual void nodeStateChanged(int nodeId, NodeState state) = 0;
		virtual void nodeEmcy(int nodeId, int errorCode, int errorRegister) = 0;
	};

	class PDOListener
	{
	public:
		PDOListener() {};
		virtual ~PDOListener() {};
		virtual void pdoReceived(int nodeId, int pdoId, const uint8* data, int len) = 0;
	};

	class CanOpenInterface
	{
	public:
		CanOpenInterface() {};
		virtual ~CanOpenInterface() {};

		virtual void setNodeMode(int nodeId, NodeMode mode) = 0;
		virtual NodeState queryNodeState(int nodeId) = 0;
		virtual void addNodeStateListener(int nodeId, NodeStateListener* listener) = 0;
		virtual void removeNodeStateListener(int nodeId, NodeStateListener* listener) = 0;

		virtual int writeSDO(int nodeId, int index, int subIndex, int value, int len) = 0;
		virtual int readSDO(int nodeId, int index, int subIndex, int& value, int& len) = 0;

		virtual void addPDOListener(int nodeId, int pdoId, PDOListener* listener) = 0;
		virtual void removePDOListener(int nodeId, int pdoId, PDOListener* listener) = 0;
		virtual void writePDO(int nodeId, int pdoId, uint8* data, int len) = 0;

		virtual void sendSYNC() = 0;
	};

}


#endif /* CANOPENMASTERDEVICE_HPP_ */
