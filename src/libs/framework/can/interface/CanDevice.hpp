/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANDEVICE_HPP_
#define CANDEVICE_HPP_

#include "CanMessage.hpp"
#include "CanListener.hpp"

#include <rcc/Device.hpp>

namespace can
{

	class CanDevice
	{
	public:
		CanDevice() {};
		virtual ~CanDevice() {};

		virtual bool sendCanMessage(const CanMessage* message) = 0;
		virtual void addCanListener(CanListener* listener) = 0;
		virtual void addCanListener(CanListener* listener, uint16 message_id) = 0;
		virtual void removeCanListener(CanListener* listener) = 0;
		virtual void removeCanListener(CanListener* listener, uint16 message_id) = 0;

		// This method is also defined by the Device class
		virtual RPI::DeviceState getDeviceState() const = 0;
	};

} /* namespace can */
#endif /* CANDEVICE_HPP_ */
