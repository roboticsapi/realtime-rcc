/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SocketCanDevice.hpp"
#include <rtt/Logger.hpp>
#include <fcntl.h>
#include <rtt/Activity.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

namespace socketcan
{
	using namespace std;

	SocketCanDevice::SocketCanDevice(std::string name, RPI::parameter_t parameters) :
			Device(name, parameters), TaskContext(name, PreOperational)
	{
		devicename = getParameter("devicename", "can0");
		socketFileDescriptor = -1;
		wasConnected = true;
		triggerInitialization(true);

		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0, "SocketCan"));
	}

	SocketCanDevice::~SocketCanDevice()
	{
		stop();
		if (socketFileDescriptor != -1)
			close(socketFileDescriptor);
	}

	void SocketCanDevice::triggerInitialization(bool _firstConnectAttempt)
	{
		RTT::os::MutexLock lock(canMutex);

		init = true;
		firstConnectAttempt = _firstConnectAttempt;
		if (socketFileDescriptor != -1)
		{
			close(socketFileDescriptor);
			socketFileDescriptor = -1;
		}
	}

	void SocketCanDevice::initialize()
	{
		RTT::os::MutexLock lock(canMutex);

		/* Create the socket */
		socketFileDescriptor = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (socketFileDescriptor == -1)
		{
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Can not create socket" << RTT::endlog();
			return;
		}

		/* Locate the interface you wish to use */
		struct ifreq ifr;
		strncpy(ifr.ifr_name, devicename.c_str(), IFNAMSIZ);

		int ioop = ioctl(socketFileDescriptor, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled with that device's index */
		if (ioop == -1)
		{
			close(socketFileDescriptor);
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Can not initialize can device '" << devicename << "'" << RTT::endlog();
			return;
		}

		/* Select that CAN interface, and bind the socket to it. */
		struct sockaddr_can addr;
		// TODO:
//		addr.can_addr.tp.rx_id.addr. = ;
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		int bop = ::bind(socketFileDescriptor, (struct sockaddr*) &addr, sizeof(addr));
		if (bop == -1)
		{
			close(socketFileDescriptor);
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Can not bind to '" << devicename << "'" << RTT::endlog();
			return;
		}

		if (!firstConnectAttempt) RTT::log(RTT::Info) << "Successfully reconnected to '" << devicename << "'" << RTT::endlog();
		init = false;
		wasConnected = true;
	}

	bool SocketCanDevice::startHook()
	{
		return true;
	}

	void SocketCanDevice::updateHook()
	{
		if (init)
		{
			initialize();
			firstConnectAttempt = false;
			trigger();
			return;
		}

		/* Read a message back from the CAN bus */
		fd_set set;
		FD_ZERO(&set);
		FD_SET(socketFileDescriptor, &set);

		timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100;

		int rd = select(socketFileDescriptor + 1, &set, NULL, NULL, &timeout);
		if (rd == 1)
		{
			struct can_frame frame;
			int bytes_read = read(socketFileDescriptor, &frame, sizeof(frame));
			if (bytes_read == -1)
			{
				close(socketFileDescriptor);
				triggerInitialization(false);
				if (wasConnected)
					RTT::log(RTT::Error) << "Failed reading from '" << devicename << "'" << RTT::endlog();
				wasConnected = false;
				trigger();
				return;
			} else
			{
				if (!wasConnected)
				{
					RTT::log(RTT::Info) << "Succeeded reading from '" << devicename << "'" << RTT::endlog();
				}
				wasConnected = true;
			}

			if (bytes_read != sizeof(frame))
			{
				RTT::log(RTT::Error) << "Read" << bytes_read << " instead of " << sizeof(frame) << " bytes from '"
						<< devicename << "'" << RTT::endlog();
				trigger();
				return;
			}

			receivedMessage.message_id = frame.can_id;
			receivedMessage.rtr = 0;
			receivedMessage.length = frame.can_dlc;
			memcpy(receivedMessage.data, frame.data, frame.can_dlc);
			notifyListeners(&receivedMessage);
		}

		trigger();
	}

	void SocketCanDevice::notifyListeners(const can::CanMessage* message)
	{
		// Listeners just for this message id
		notifyListeners(canListeners[message->message_id + 1], message);

		// Listeners for all message ids
		notifyListeners(canListeners[0], message);
	}

	void SocketCanDevice::notifyListeners(list<can::CanListener*>* lst, const can::CanMessage* message)
	{
		if (lst != NULL)
		{
			for (list<can::CanListener*>::const_iterator it = lst->begin(); it != lst->end(); ++it)
			{
				(*it)->dataReceived(message);
			}
		}
	}

	bool SocketCanDevice::sendCanMessage(const can::CanMessage* message)
	{
		if (getDeviceState() != RPI::DeviceState::OPERATIONAL)
			return false;

		fd_set set;
		FD_ZERO(&set);
		FD_SET(socketFileDescriptor, &set);

		timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100;

		int rd = select(socketFileDescriptor + 1, NULL, &set, NULL, &timeout);
		if (rd != 1)
			return false;

		/* Send a message to the CAN bus */
		struct can_frame frame;
		frame.can_id = message->message_id;
		frame.can_dlc = message->length;
		memcpy(frame.data, message->data, can::CanMessageDataSize);

		int bytes_sent = write(socketFileDescriptor, &frame, sizeof(frame));
		if (bytes_sent == -1)
		{
			close(socketFileDescriptor);
			triggerInitialization(false);
			if (wasConnected)
				RTT::log(RTT::Error) << "Error writing to '" << devicename << "', ID " << message->message_id << RTT::endlog();
			wasConnected = false;
			return false;
		} else
		{
			if (!wasConnected)
			{
				RTT::log(RTT::Info) << "Succeeded writing to '" << devicename << "', ID " << message->message_id << RTT::endlog();
			}
			wasConnected = true;
		}

		if (bytes_sent != sizeof(frame))
		{
			RTT::log(RTT::Error) << "Wrote " << bytes_sent << " instead of " << sizeof(frame) << " bytes to '"
					<< devicename << "'" << RTT::endlog();
			return false;
		}

		return true;
	}

	void SocketCanDevice::addCanListener(can::CanListener* listener)
	{
		RTT::os::MutexLock lock(canMutex);

		if (canListeners[0] == NULL)
		{
			canListeners[0] = new list<can::CanListener*>();
		}
		canListeners[0]->push_back(listener);
	}

	void SocketCanDevice::addCanListener(can::CanListener* listener, uint16 message_id)
	{
		RTT::os::MutexLock lock(canMutex);

		if (canListeners[message_id + 1] == NULL)
		{
			canListeners[message_id + 1] = new list<can::CanListener*>();
		}
		canListeners[message_id + 1]->push_back(listener);
	}

	void SocketCanDevice::removeCanListener(can::CanListener* listener)
	{
		RTT::os::MutexLock lock(canMutex);

		list<can::CanListener*>* lst = canListeners[0];
		if (lst == NULL)
			return;
		lst->remove(listener);
		if (lst->size() != 0)
			return;
		canListeners[0] = NULL;
		delete lst;
	}

	void SocketCanDevice::removeCanListener(can::CanListener* listener, uint16 message_id)
	{
		RTT::os::MutexLock lock(canMutex);

		list<can::CanListener*>* lst = canListeners[message_id + 1];
		if (lst == NULL)
			return;
		lst->remove(listener);
		if (lst->size() != 0)
			return;
		canListeners[message_id + 1] = NULL;
		delete lst;
	}

	SocketCanDevice* SocketCanDevice::createDevice(std::string name, RPI::parameter_t parameters)
	{
		SocketCanDevice* ret = new SocketCanDevice(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	void SocketCanDevice::updateParameters()
	{

	}

	std::set<std::string> SocketCanDevice::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void SocketCanDevice::setEStop(bool estop)
	{

	}

	RPI::DeviceState SocketCanDevice::getDeviceState() const
	{
		if (!wasConnected)
			return RPI::DeviceState::OFFLINE;
		return RPI::DeviceState::OPERATIONAL;
	}

} /* namespace socketcan */
