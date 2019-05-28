/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef VIRTUALRCCDEVICE_HPP_
#define VIRTUALRCCDEVICE_HPP_

#include "Device.hpp"
#include <rtt/os/TimeService.hpp>
#include <rtt/TaskContext.hpp>

#define VRCC_CYCLE_TIME 0.01

namespace RPI
{
	const std::string dev_virtualrcc = "rcc";

	class VirtualRCCDevice: public Device, public RTT::TaskContext
	{
	public:
		VirtualRCCDevice(std::string name, RPI::parameter_t parameter);
		virtual ~VirtualRCCDevice();

		std::set<std::string> getMutableParameters() const;
		void updateParameters();
		bool isRemovable()
		{
			return false;
		}

		bool getNetUnloading() const;
		int getDebuggingLevel() const;
		double getDebuggingTime() const;

		void setEStop(bool value);
		bool getEStop() const;

		std::string getWebPort() const;
		std::string getDIOPort() const;

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		DeviceState getDeviceState() const;
	private:
		bool netUnloading;
		int debuggingLevel;
		double debuggingTime;

		RTT::os::TimeService::nsecs lastEStopEvent;
		RTT::os::TimeService::nsecs lastEStopTriggered;
		bool isEStop;
		bool needEStop;

		std::string webport, dioport;

		void notifyDevices();
	};

} /* namespace RPI */
#endif /* VIRTUALRCCDEVICE_HPP_ */
