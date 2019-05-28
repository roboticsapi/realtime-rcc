/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANCANOPENDEVICE_HPP_
#define CANCANOPENDEVICE_HPP_

#include "../interface/CanOpenInterface.hpp"
#include "../../can/interface/CanDevice.hpp"
#include <rcc/DeviceInstanceT.hpp>

namespace canopen
{
	const std::string dev_canopen = "canopen";

	class CanOpenDevice: public CanOpenInterface, public virtual RPI::Device, public can::CanListener
	{
	public:
		CanOpenDevice(std::string name, RPI::parameter_t parameters);
		virtual ~CanOpenDevice();

		// CANListener
		virtual void dataReceived(const can::CanMessage* message);

		// RPI::Device
		static CanOpenDevice* createDevice(std::string name, RPI::parameter_t parameters);
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		RPI::DeviceState getDeviceState() const;
		void setEStop(bool estop);

		// CanOpenMasterDevice
		void setNodeMode(int nodeId, NodeMode mode);
		NodeState queryNodeState(int nodeId);
		void addNodeStateListener(int nodeId, NodeStateListener* listener);
		void removeNodeStateListener(int nodeId, NodeStateListener* listener);

		int writeSDO(int nodeId, int index, int subIndex, int value, int len);
		int readSDO(int nodeId, int index, int subIndex, int& value, int& len);

		void addPDOListener(int nodeId, int pdoId, PDOListener* listener);
		void removePDOListener(int nodeId, int pdoId, PDOListener* listener);
		void writePDO(int nodeId, int pdoId, uint8* data, int len);

		void sendSYNC();

	private:
		RPI::DeviceInstanceT<can::CanDevice> canBus;
		can::CanMessage buildMessage(int cobId, int rtr, int len, char d1, char d2, char d3, char d4, char d5, char d6, char d7, char d8);
		std::map<int, std::list<NodeStateListener*> > nodeStateListeners;
		std::map<int, std::list<PDOListener*> > pdoListeners;
	};

} /* namespace canopen */
#endif /* CANCANOPENDEVICE_HPP_ */
