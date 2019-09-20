/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "RtSocketCanDevice.hpp"
#include <rtt/Logger.hpp>
#include <rtt/Activity.hpp>

#include <rtdm/rtcan.h>
#include <string.h>

namespace rtsocketcan
{
	using namespace std;

	RtSocketCanDevice::RtSocketCanDevice(std::string name, RPI::parameter_t parameters) :
			Device(name, parameters), TaskContext(name, PreOperational)
	{
		devicename = getParameter("devicename", "rtcan0");
		socketFileDescriptor = -1;
		wasConnected = true;
		triggerInitialization(true);

		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0, "RtSocketCan"));
	}

	RtSocketCanDevice::~RtSocketCanDevice()
	{
		stop();
		if (socketFileDescriptor != -1)
			rt_dev_close(socketFileDescriptor);
	}

	void RtSocketCanDevice::triggerInitialization(bool _firstConnectAttempt)
	{
		RTT::os::MutexLock lock(canMutex);

		init = true;
		firstConnectAttempt = _firstConnectAttempt;
		if (socketFileDescriptor != -1)
		{
			rt_dev_close(socketFileDescriptor);
			socketFileDescriptor = -1;
		}
	}

	void RtSocketCanDevice::initialize()
	{
		RTT::os::MutexLock lock(canMutex);

		/* Create the socket */
		socketFileDescriptor = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (socketFileDescriptor < 0)
		{
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Can not create socket" << RTT::endlog();
			return;
		}

		/* Locate the interface you wish to use */
		struct ifreq ifr;
		strncpy(ifr.ifr_name, devicename.c_str(), IFNAMSIZ);
		int ioop = rt_dev_ioctl(socketFileDescriptor, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled with that device's index */
		if (ioop < 0)
		{
			rt_dev_close(socketFileDescriptor);
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Can not initialize can device '" << devicename << "'" << RTT::endlog();
			return;
		}

		/* Select that CAN interface, and bind the socket to it. */
		struct sockaddr_can addr;

		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		int bop = rt_dev_bind(socketFileDescriptor, (struct sockaddr*) &addr, sizeof(addr));
		if (bop < 0)
		{
			rt_dev_close(socketFileDescriptor);
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Can not bind to '" << devicename << "'" << RTT::endlog();
			return;
		}

		/* Set timeout for recv operation */
		nanosecs_rel_t rtimeout = 1e6; // 1ms timeout for receive
		int recvtimeoutret = rt_dev_ioctl(socketFileDescriptor, RTCAN_RTIOC_RCV_TIMEOUT, &rtimeout);
		if (recvtimeoutret)
		{
			rt_dev_close(socketFileDescriptor);
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Cannot set timeout on '" << devicename << "'" << RTT::endlog();
			return;

		}

		int sendtimeoutret = rt_dev_ioctl(socketFileDescriptor, RTCAN_RTIOC_SND_TIMEOUT, &rtimeout);
		if (sendtimeoutret)
		{
			rt_dev_close(socketFileDescriptor);
			if (firstConnectAttempt)
				RTT::log(RTT::Error) << "Cannot set timeout on '" << devicename << "'" << RTT::endlog();
			return;

		}

//		if (!firstConnectAttempt) RTT::log(RTT::Info) << "Successfully reconnected to '" << devicename << "'" << RTT::endlog();
		init = false;
	}

	bool RtSocketCanDevice::startHook()
	{
		return true;
	}

	void RtSocketCanDevice::updateHook()
	{
		if (init)
		{
			initialize();
			trigger();
			return;
		}

		struct sockaddr_can addr;
		socklen_t addrlen = sizeof(addr);
		struct can_frame frame;

		int bytes_read = rt_dev_recvfrom(socketFileDescriptor, &frame, sizeof(can_frame_t), 0,
				(struct sockaddr *) &addr, &addrlen);
		if (bytes_read == -ETIMEDOUT)
		{
			trigger();
			return;
		} else if (bytes_read < 0)
		{
			rt_dev_close(socketFileDescriptor);
			triggerInitialization(false);
			if (wasConnected)
				RTT::log(RTT::Error) << "Lost connection to '" << devicename << "' with " << bytes_read
						<< RTT::endlog();
			wasConnected = false;
			trigger();
			return;

		} else
		{
			if (!wasConnected)
			{
				RTT::log(RTT::Info) << "Successfully reconnected to '" << devicename << "'" << RTT::endlog();
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

		trigger();
	}

	void RtSocketCanDevice::notifyListeners(const can::CanMessage* message)
	{
		// Listeners just for this message id
		notifyListeners(canListeners[message->message_id + 1], message);

		// Listeners for all message ids
		notifyListeners(canListeners[0], message);
	}

	void RtSocketCanDevice::notifyListeners(list<can::CanListener*>* lst, const can::CanMessage* message)
	{
		if (lst != NULL)
		{
			for (list<can::CanListener*>::const_iterator it = lst->begin(); it != lst->end(); ++it)
			{
				(*it)->dataReceived(message);
			}
		}
	}

	bool RtSocketCanDevice::sendCanMessage(const can::CanMessage* message)
	{
		if (getDeviceState() != RPI::DeviceState::OPERATIONAL)
			return false;

		/* Send a message to the CAN bus */
		struct can_frame frame;
		frame.can_id = message->message_id;
		frame.can_dlc = message->length;
		memcpy(frame.data, message->data, can::CanMessageDataSize);

		int bytes_sent = rt_dev_send(socketFileDescriptor, &frame, sizeof(frame), 0);
		if (bytes_sent == -ETIMEDOUT)
		{
			return false;
		} else if (bytes_sent < 0)
		{
			rt_dev_close(socketFileDescriptor);
			triggerInitialization(false);
			if (wasConnected)
				RTT::log(RTT::Error) << "Lost connection to '" << devicename << "' with " << bytes_sent
						<< RTT::endlog();
			wasConnected = false;
			return false;
		} else
		{
			if (!wasConnected)
			{
				RTT::log(RTT::Info) << "Successfully reconnected to '" << devicename << "'" << RTT::endlog();
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

	void RtSocketCanDevice::addCanListener(can::CanListener* listener)
	{
		RTT::os::MutexLock lock(canMutex);

		if (canListeners[0] == NULL)
		{
			canListeners[0] = new list<can::CanListener*>();
		}
		canListeners[0]->push_back(listener);
	}

	void RtSocketCanDevice::addCanListener(can::CanListener* listener, uint16 message_id)
	{
		RTT::os::MutexLock lock(canMutex);

		if (canListeners[message_id + 1] == NULL)
		{
			canListeners[message_id + 1] = new list<can::CanListener*>();
		}
		canListeners[message_id + 1]->push_back(listener);
	}

	void RtSocketCanDevice::removeCanListener(can::CanListener* listener)
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

	void RtSocketCanDevice::removeCanListener(can::CanListener* listener, uint16 message_id)
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

	RtSocketCanDevice* RtSocketCanDevice::createDevice(std::string name, RPI::parameter_t parameters)
	{
		RtSocketCanDevice* ret = new RtSocketCanDevice(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	void RtSocketCanDevice::updateParameters()
	{

	}

	std::set<std::string> RtSocketCanDevice::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void RtSocketCanDevice::setEStop(bool estop)
	{

	}

	RPI::DeviceState RtSocketCanDevice::getDeviceState() const
	{
		if (!wasConnected)
			return RPI::DeviceState::OFFLINE;
		return RPI::DeviceState::OPERATIONAL;
	}

} /* namespace socketcan */
