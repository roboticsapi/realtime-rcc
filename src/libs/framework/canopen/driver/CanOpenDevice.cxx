/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "CanOpenDevice.hpp"
#include <iostream>

namespace canopen
{

	class WaitForCanMessage: public can::CanListener
	{
	public:
		WaitForCanMessage(can::CanDevice* can, int cobId) :
				cobId(cobId), can(can), received(false)
		{
			can->addCanListener(this);
		}

		~WaitForCanMessage()
		{
			can->removeCanListener(this);
		}
		virtual void dataReceived(const can::CanMessage* message)
		{
			if (message->message_id == cobId)
			{
				msg = *message;
				received = true;
			}
		}
		can::CanMessage waitForMessage()
		{
			while (!received)
			{
				usleep(20);
			}
			return msg;
		}

	private:
		int cobId;
		can::CanDevice* can;
		can::CanMessage msg;
		bool received;
	};

	CanOpenDevice::CanOpenDevice(std::string name, RPI::parameter_t parameters) :
			RPI::Device(name, parameters), canBus(getParameter("candevice", ""))
	{
		canBus.getDevice()->addCanListener(this);

	}

	void CanOpenDevice::updateParameters()
	{
		// do nothing
	}

	std::set<std::string> CanOpenDevice::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	RPI::DeviceState CanOpenDevice::getDeviceState() const
	{
		return canBus.getDevice()->getDeviceState();
	}

	CanOpenDevice::~CanOpenDevice()
	{
		canBus.getDevice()->removeCanListener(this);
	}

	void CanOpenDevice::setEStop(bool estop)
	{
	}

	CanOpenDevice* CanOpenDevice::createDevice(std::string name, RPI::parameter_t parameters)
	{
		return new CanOpenDevice(name, parameters);
	}

	can::CanMessage CanOpenDevice::buildMessage(int cobId, int rtr, int len, char d1, char d2, char d3, char d4,
			char d5, char d6, char d7, char d8)
	{
		can::CanMessage ret;
		ret.message_id = cobId;
		ret.rtr = rtr;
		ret.length = len;
		ret.data[0] = d1;
		ret.data[1] = d2;
		ret.data[2] = d3;
		ret.data[3] = d4;
		ret.data[4] = d5;
		ret.data[5] = d6;
		ret.data[6] = d7;
		ret.data[7] = d8;
		return ret;
	}

	void CanOpenDevice::dataReceived(const can::CanMessage* message)
	{
		int nodeId = message->message_id & 0x7F;
//		std::cout << "Received message " << message->message_id << "." << std::endl;
		switch (message->message_id >> 7)
		{
		case 0x01: // EMCY
			for (std::list<NodeStateListener*>::iterator it = nodeStateListeners[nodeId].begin();
					it != nodeStateListeners[nodeId].end(); ++it)
			{
				(*it)->nodeEmcy(nodeId, (int)message->data[0] + (message->data[1] << 8), message->data[2]);
			}
			break;
		case 0x03: // TxPDO 1
		case 0x05: // TxPDO 2
		case 0x07: // TxPDO 3
		case 0x09: // TxPDO 4
		{
			int pdoId = message->message_id >> 8;
			for (std::list<PDOListener*>::iterator it = pdoListeners[message->message_id].begin();
					it != pdoListeners[message->message_id].end(); ++it)
			{
				(*it)->pdoReceived(nodeId, pdoId, &message->data[0], message->length);
			}
//			std::cout << "TXPDO" << std::endl;
			break;
		}
		case 0x04: // RxPDO 1
		case 0x06: // RxPDO 2
		case 0x08: // RxPDO 3
		case 0x0A: // RxPDO 4
//			std::cout << "RXPDO" << std::endl;
			break;
		case 0x0B: // TxSDO
		case 0x0C: // RxSDO
//			std::cout << "SDO" << std::endl;
			break;
		case 0x0D: // NMT Error
//			std::cout << "NMT Error" << std::endl;
			break;
		case 0x0E: // NMT (Node status)
		{
//			std::cout << "Node status" << std::endl;
			NodeState state = NS_BOOTUP;
			switch (message->data[0] & 0x7f)
			{
			case 0x00:
				state = NS_BOOTUP;
				break;
			case 0x04:
				state = NS_STOPPED;
				break;
			case 0x05:
				state = NS_OPERATIONAL;
				break;
			case 0x7F:
				state = NS_PRE_OPERATIONAL;
				break;
			}
			for (std::list<NodeStateListener*>::iterator it = nodeStateListeners[0].begin();
					it != nodeStateListeners[0].end(); ++it)
			{
				(*it)->nodeStateChanged(nodeId, state);
			}
			for (std::list<NodeStateListener*>::iterator it = nodeStateListeners[nodeId].begin();
					it != nodeStateListeners[nodeId].end(); ++it)
			{
				(*it)->nodeStateChanged(nodeId, state);
			}
			break;
		}
		}
	}

	void CanOpenDevice::setNodeMode(int nodeId, NodeMode mode)
	{
		can::CanMessage message = buildMessage(0, 0, 2, mode, nodeId, 0, 0, 0, 0, 0, 0);
		canBus.getDevice()->sendCanMessage(&message);
	}

	// TODO: This method does not seem to work, the Schunk PBA does not respond.
	NodeState CanOpenDevice::queryNodeState(int nodeId)
	{
		can::CanMessage message = buildMessage(0x700 + nodeId, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		WaitForCanMessage wait(canBus.getDevice(), 0x700 + nodeId);
		canBus.getDevice()->sendCanMessage(&message);
		message = wait.waitForMessage();
		switch (message.data[0] & 0x7f)
		{
		case 0x04:
			return NS_STOPPED;
		case 0x05:
			return NS_OPERATIONAL;
		case 0x7F:
			return NS_PRE_OPERATIONAL;
		case 0x00:
			return NS_BOOTUP;
		default:
			std::cout << "Error: Node state " << message.data[0] << std::endl;
			return NS_BOOTUP;
		}
	}

	void CanOpenDevice::addNodeStateListener(int nodeId, NodeStateListener* listener)
	{
		nodeStateListeners[nodeId].push_back(listener);
	}
	void CanOpenDevice::removeNodeStateListener(int nodeId, NodeStateListener* listener)
	{
		nodeStateListeners[nodeId].remove(listener);
	}

	int CanOpenDevice::writeSDO(int nodeId, int index, int subIndex, int value, int len)
	{
		can::CanMessage message;
		message.message_id = 0x600 + nodeId;
		message.rtr = 0;
		message.length = 8;
		message.data[1] = index & 0xff;
		message.data[2] = ((index >> 8) & 0xff);
		message.data[3] = subIndex;
		message.data[4] = value & 0xff;
		message.data[5] = (value >> 8) & 0xff;
		message.data[6] = (value >> 16) & 0xff;
		message.data[7] = (value >> 24) & 0xff;
		switch (len)
		{
		case 1:
			message.data[0] = 0x2f;
			break;
		case 2:
			message.data[0] = 0x2b;
			break;
		case 3:
			message.data[0] = 0x27;
			break;
		case 4:
			message.data[0] = 0x23;
			break;
		}
		WaitForCanMessage wait(canBus.getDevice(), 0x580 + nodeId);
		canBus.getDevice()->sendCanMessage(&message);
		message = wait.waitForMessage();
		if (message.data[0] == 0x60)
			return 0;
		if (message.data[0] == 0x80) {
			std::cout << std::hex << (message.data[4] + (message.data[5] << 8) + (message.data[6] << 16) + (message.data[7] << 24)) << std::endl;
			return message.data[4] + (message.data[5] << 8) + (message.data[6] << 16) + (message.data[7] << 24);
		}
		std::cout << "Invalid response: " << message.data[0] << std::endl;
		return -1;
	}

	int CanOpenDevice::readSDO(int nodeId, int index, int subIndex, int& value, int& len)
	{
		can::CanMessage message = buildMessage(0x600 + nodeId, 0, 8, 0x40, index & 0xff, (index >> 8) & 0xff, subIndex,
				0, 0, 0, 0);
		WaitForCanMessage wait(canBus.getDevice(), 0x580 + nodeId);
		canBus.getDevice()->sendCanMessage(&message);
		message = wait.waitForMessage();
		if (message.data[0] == 0x4f)
		{
			len = 1;
			value = message.data[4];
			return 0;
		} else if (message.data[0] == 0x4b)
		{
			len = 2;
			value = message.data[4] + (message.data[5] << 8);
			return 0;
		} else if (message.data[0] == 0x47)
		{
			len = 3;
			value = message.data[4] + (message.data[5] << 8) + (message.data[6] << 16);
			return 0;
		} else if (message.data[0] == 0x43)
		{
			len = 4;
			value = message.data[4] + (message.data[5] << 8) + (message.data[6] << 16) + (message.data[7] << 24);
			return 0;
		} else if (message.data[0] == 0x80)
		{
			len = 0;
			value = 0;
			return message.data[4] + (message.data[5] << 8) + (message.data[6] << 16) + (message.data[7] << 24);
		}
		std::cout << "Invalid response: " << message.data[0] << std::endl;
		return -1;
	}

	void CanOpenDevice::addPDOListener(int nodeId, int pdoId, PDOListener* listener)
	{
		//int cobId = 0x80 + 0x100 * pdoId + nodeId;
		int cobId = 0x080 + (pdoId << 8) + nodeId;
		pdoListeners[cobId].push_back(listener);
	}
	void CanOpenDevice::removePDOListener(int nodeId, int pdoId, PDOListener* listener)
	{
		//int cobId = 0x80 + 0x100 * pdoId + nodeId;
		int cobId = 0x080 + (pdoId << 8) + nodeId;
		pdoListeners[cobId].remove(listener);
	}
	void CanOpenDevice::writePDO(int nodeId, int pdoId, uint8* data, int len)
	{
		//int cobId = 0x100 + 0x100 * pdoId + nodeId;
		int cobId = 0x100 + (pdoId << 8) + nodeId;
		can::CanMessage message = buildMessage(cobId, 0, len, data[0], data[1], data[2], data[3], data[4], data[5],
				data[6], data[7]);
		canBus.getDevice()->sendCanMessage(&message);
	}

	void CanOpenDevice::sendSYNC()
	{
		can::CanMessage message = buildMessage(0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		canBus.getDevice()->sendCanMessage(&message);
	}

} /* namespace canopen */
