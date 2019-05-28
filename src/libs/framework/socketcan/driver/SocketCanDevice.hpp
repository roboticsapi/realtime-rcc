/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SOCKETCANDEVICE_HPP_
#define SOCKETCANDEVICE_HPP_

#include <rcc/Device.hpp>
#include <rcc/DeviceFactory.hpp>
#include "../../can/interface/CanDevice.hpp"
#include "../../can/interface/CanListener.hpp"
#include "../../can/interface/CanMessage.hpp"

#include <rtt/os/Mutex.hpp>
#include <rtt/TaskContext.hpp>
#include <list>
#include <map>

using std::list;
using std::map;

namespace socketcan
{
	typedef unsigned char uint8;
	typedef unsigned short uint16;

	const std::string dev_can = "socketcan";

	class SocketCanDevice: public RPI::Device, public can::CanDevice, public RTT::TaskContext
	{
	public:
		SocketCanDevice(std::string name, RPI::parameter_t parameters);
		virtual ~SocketCanDevice();

		static SocketCanDevice* createDevice(std::string name, RPI::parameter_t parameters);
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		RPI::DeviceState getDeviceState() const;

		void setEStop(bool estop);

		bool startHook();
		void updateHook();

		bool sendCanMessage(const can::CanMessage* message);
		void addCanListener(can::CanListener* listener);
		void addCanListener(can::CanListener* listener, uint16 message_id);
		void removeCanListener(can::CanListener* listener);
		void removeCanListener(can::CanListener* listener, uint16 message_id);

	private:
		RTT::os::Mutex canMutex;

		std::string devicename;
		int socketFileDescriptor;
		bool init;
		can::CanMessage receivedMessage;
		bool firstConnectAttempt;
		bool wasConnected;

		// map of lists of all listeners who get notified about received CAN messages
		// in the '0'-th list, all listeners for no specific message_id are listed
		// in the (n+1)-th list, all listeners for message_id 'n' are stored
		map<uint16, list<can::CanListener*>*> canListeners;

		void triggerInitialization(bool firstConnectAttempt);
		void initialize();

		void notifyListeners(const can::CanMessage* message);
		void notifyListeners(list<can::CanListener*>* list, const can::CanMessage* message);
	};

} /* namespace socketcan */
#endif /* SOCKETCANDEVICE_HPP_ */
