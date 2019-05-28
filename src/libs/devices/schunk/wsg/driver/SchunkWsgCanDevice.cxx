/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SchunkWsgCanDevice.hpp"
#include <rtt/extras/PeriodicActivity.hpp>
#include <rtt/Logger.hpp>


namespace schunkwsg
{

	SchunkWsgCanDevice::SchunkWsgCanDevice(std::string name, RPI::parameter_t parameters):
			Device(name, parameters), SchunkWsgDevice(name, parameters), canBus(getParameter("candevice", "")), TaskContext(name, PreOperational)
	{
		int id_write = atoi(getParameter("id_write", "100").c_str());
		int id_read = atoi(getParameter("id_read", "101").c_str());

		sendStop = false;
		sendFastStop = false;

		deviceState = RPI::DeviceState::OFFLINE;
		eStop = false;

		currentOpeningWidth = 0;
		currentVelocity = 0;
		currentForce = 0;
		currentGraspingState = IDLE;

		triggerInitialization();

		can::CanDevice* canDevice = canBus.getDevice();
		if (!canDevice) {
			RTT::log(RTT::Error) << "No CAN device set for schunk WSG" << RTT::endlog();
			return;
		}

		ioHandler = new IO(canDevice, this, id_write, id_read);
		this->setActivity(new RTT::extras::PeriodicActivity(RTT::os::LowestPriority, 0.01));
	}

	SchunkWsgCanDevice::~SchunkWsgCanDevice()
	{
		while (!ioHandler->isTerminated()) ioHandler->cancelCommand();
		stop();
		if (ioHandler) delete(ioHandler);
	}

	void SchunkWsgCanDevice::triggerInitialization() {
		init = true;
		init_step = 0;
		busy = false;
		notInterruptableSendAction = false;
		status_code = E_SUCCESS;
	}

	void SchunkWsgCanDevice::messageReceived(uint8 command_id, StatusCode _status_code, const uint8* payload, uint16 size)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);

		// Acknowledge of a fast stop command
		if (command_id==0x23 && _status_code==E_SUCCESS) {
			deviceState = RPI::DeviceState::SAFE_OPERATIONAL;
			return;
		}

		// Acknowledge of a fast stop acknowledgment command
		if (command_id==0x24 && _status_code==E_SUCCESS) {
			deviceState = RPI::DeviceState::OPERATIONAL;
			return;
		}

		// Receiving the current opening width
		if (command_id==0x43 && _status_code==E_SUCCESS) {
			uint32 mem = payload[0] + (payload[1] * 0x100) + (payload[2] * 0x10000) + (payload[3] * 0x1000000);
			currentOpeningWidth = *((float*)&mem) / 1000.0;
			return;
		}

		// Receiving the current speed
		if (command_id==0x44 && _status_code==E_SUCCESS) {
			uint32 mem = payload[0] + (payload[1] * 0x100) + (payload[2] * 0x10000) + (payload[3] * 0x1000000);
			currentVelocity = *((float*)&mem) / 1000.0;
			return;
		}

		// Receiving the current force
		if (command_id==0x45 && _status_code==E_SUCCESS) {
			uint32 mem = payload[0] + (payload[1] * 0x100) + (payload[2] * 0x10000) + (payload[3] * 0x1000000);
			currentForce = *((float*)&mem);
			return;
		}

		// Receiving the current grasping state
		if (command_id==0x41 && _status_code==E_SUCCESS) {
			currentGraspingState = (GraspingState)payload[0];
			return;
		}
	}

	bool SchunkWsgCanDevice::homing(bool openDirection)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		payload_buf[0] = openDirection ? 0x01 : 0x02;
		if (ioHandler->sendCommand(0x20, payload_buf, 1, MOVE_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::preposition(bool relative, double width, double speed, bool stopOnBlock)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		uint8 width_b[sizeof(float)];
		floatByteArrayOfDouble(width * 1000.0, width_b);
		uint8 speed_b[sizeof(float)];
		floatByteArrayOfDouble(speed * 1000.0, speed_b);

		payload_buf[0] = (stopOnBlock ? 0x02 : 0x00) + (relative ? 0x01 : 0x00);
		payload_buf[1] = width_b[0];
		payload_buf[2] = width_b[1];
		payload_buf[3] = width_b[2];
		payload_buf[4] = width_b[3];
		payload_buf[5] = speed_b[0];
		payload_buf[6] = speed_b[1];
		payload_buf[7] = speed_b[2];
		payload_buf[8] = speed_b[3];
		if (ioHandler->sendCommand(0x21, payload_buf, 9, MOVE_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::grasp(double width, double speed)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		uint8 width_b[sizeof(float)];
		floatByteArrayOfDouble(width * 1000.0, width_b);
		uint8 speed_b[sizeof(float)];
		floatByteArrayOfDouble(speed * 1000.0, speed_b);

		payload_buf[0] = width_b[0];
		payload_buf[1] = width_b[1];
		payload_buf[2] = width_b[2];
		payload_buf[3] = width_b[3];
		payload_buf[4] = speed_b[0];
		payload_buf[5] = speed_b[1];
		payload_buf[6] = speed_b[2];
		payload_buf[7] = speed_b[3];
		if (ioHandler->sendCommand(0x25, payload_buf, 8, MOVE_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::release(double openWidth, double speed)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		uint8 width_b[sizeof(float)];
		floatByteArrayOfDouble(openWidth * 1000.0, width_b);
		uint8 speed_b[sizeof(float)];
		floatByteArrayOfDouble(speed * 1000.0, speed_b);

		payload_buf[0] = width_b[0];
		payload_buf[1] = width_b[1];
		payload_buf[2] = width_b[2];
		payload_buf[3] = width_b[3];
		payload_buf[4] = speed_b[0];
		payload_buf[5] = speed_b[1];
		payload_buf[6] = speed_b[2];
		payload_buf[7] = speed_b[3];
		if (ioHandler->sendCommand(0x26, payload_buf, 8, MOVE_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::setAcceleration(double acceleration)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		uint8 acceleration_b[sizeof(float)];
		floatByteArrayOfDouble(acceleration * 1000.0, acceleration_b);

		payload_buf[0] = acceleration_b[0];
		payload_buf[1] = acceleration_b[1];
		payload_buf[2] = acceleration_b[2];
		payload_buf[3] = acceleration_b[3];
		if (ioHandler->sendCommand(0x30, payload_buf, 4, STATISTICS_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::setForceLimit(double force)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		uint8 force_b[sizeof(float)];
		floatByteArrayOfDouble(force, force_b);

		payload_buf[0] = force_b[0];
		payload_buf[1] = force_b[1];
		payload_buf[2] = force_b[2];
		payload_buf[3] = force_b[3];
		if (ioHandler->sendCommand(0x32, payload_buf, 4, STATISTICS_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::setSoftLimits(double inner, double outer)
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		uint8 inner_b[sizeof(float)];
		floatByteArrayOfDouble(inner * 1000.0, inner_b);
		uint8 outer_b[sizeof(float)];
		floatByteArrayOfDouble(outer * 1000.0, outer_b);

		payload_buf[0] = inner_b[0];
		payload_buf[1] = inner_b[1];
		payload_buf[2] = inner_b[2];
		payload_buf[3] = inner_b[3];
		payload_buf[4] = outer_b[0];
		payload_buf[5] = outer_b[1];
		payload_buf[6] = outer_b[2];
		payload_buf[7] = outer_b[3];
		if (ioHandler->sendCommand(0x34, payload_buf, 8, STATISTICS_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::clearSoftLimits()
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		if (ioHandler->sendCommand(0x36, payload_buf, 0, STATISTICS_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	void SchunkWsgCanDevice::stopDevice()
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		sendStop = true;
	}

	bool SchunkWsgCanDevice::acknowledgeFaststopOrFault()
	{
		RTT::os::MutexLock lock(schunkwsgMutex);
		if (!canSend()) return false;

		payload_buf[0] = 0x61;
		payload_buf[1] = 0x63;
		payload_buf[2] = 0x6B;
		if (ioHandler->sendCommand(0x24, payload_buf, 3, STATISTICS_COMMAND_TIMEOUT)) {
			busy = true;
			return true;
		}
		return false;
	}

	bool SchunkWsgCanDevice::isBusy() {
		RTT::os::MutexLock lock(schunkwsgMutex);
		return busy;
	}

	StatusCode SchunkWsgCanDevice::getStatusCode() {
		return status_code;
	}

	double SchunkWsgCanDevice::getOpeningWidth() {
			return currentOpeningWidth;
		}

	double SchunkWsgCanDevice::getVelocity() {
		return currentVelocity;
	}

	double SchunkWsgCanDevice::getForce() {
		return currentForce;
	}

	GraspingState SchunkWsgCanDevice::getGraspingState() {
		return currentGraspingState;
	}

	bool SchunkWsgCanDevice::canSend() {
		return (deviceState==RPI::DeviceState::OPERATIONAL && !init && !notInterruptableSendAction && !sendFastStop && !sendStop);
	}

	bool SchunkWsgCanDevice::startHook()
	{
		return true;
	}

	void SchunkWsgCanDevice::updateHook()
	{
		if (!ioHandler) return;

		RTT::os::MutexLock lock(schunkwsgMutex);

		// If device is not operational, we try to reinitialize. If device is safe_operational, we only try to reinitialize if not in estop mode!
		if (!init && (getDeviceState()==RPI::DeviceState::OFFLINE || (getDeviceState()==RPI::DeviceState::SAFE_OPERATIONAL && !eStop))) {
			ioHandler->cancelCommand();
			if (!ioHandler->isTerminated()) return;
			triggerInitialization();
		}

		bool terminated = ioHandler->isTerminated();

		// Important send action still running => continue
		if (notInterruptableSendAction && !terminated) {
			return;
		}

		// Important send action terminated
		else if (notInterruptableSendAction) {
			notInterruptableSendAction = false;
			// Important send action was not successful? => reinitialize device
			if (!ioHandler->isSuccessful()) {
				if (ioHandler->isError() && ioHandler->getStatusCode()==E_ACCESS_DENIED) {
					deviceState = RPI::DeviceState::SAFE_OPERATIONAL;
				}
				else {
					deviceState = RPI::DeviceState::OFFLINE;
				}
				triggerInitialization();
			}
		}

		// User defined send action still running
		else if (!terminated) {
		}

		// User defined send action terminated
		else {
			busy = false;
			status_code = ioHandler->getStatusCode();

			if (ioHandler->isError() && ioHandler->getStatusCode()==E_ACCESS_DENIED) {
				deviceState = RPI::DeviceState::SAFE_OPERATIONAL;
				return;
			}
			if (ioHandler->isTimeout()) {
				deviceState = RPI::DeviceState::OFFLINE;
				return;
			}
		}

		if (init) {
			uint16 time = 0;
			switch (init_step) {
			case 0:
				if (!eStop) { // Send a command which tells the device to acknowledge a faststop state
					payload_buf[0] = 0x61;
					payload_buf[1] = 0x63;
					payload_buf[2] = 0x6B;
					if (ioHandler->sendCommand(0x24, payload_buf, 3, STATISTICS_COMMAND_TIMEOUT)) {
						init_step++;
						notInterruptableSendAction = true;
					}
					return;
				}
				else { // Send a command which tells the device to enter fast stop state
					payload_buf[0] = 0x61;
					payload_buf[1] = 0x63;
					payload_buf[2] = 0x6B;
					if (ioHandler->sendCommand(0x23, payload_buf, 0, STATISTICS_COMMAND_TIMEOUT)) {
						init_step++;
						notInterruptableSendAction = true;
					}
					return;
				}
			case 1:	// Send a command which tells the device to periodically publish its opening width
				payload_buf[0] = 0x03; // 0x01 for periodically, 0x03 for periodically but only if there is a value change
				time = UPDATE_OPENING_WIDTH_EVERY_MS;
				payload_buf[1] = *(((uint8*)&time));
				payload_buf[2] = *(((uint8*)&time) + 1);
				if (ioHandler->sendCommand(0x43, payload_buf, 3, STATISTICS_COMMAND_TIMEOUT)) {
					init_step++;
					notInterruptableSendAction = true;
				}
				return;
			case 2:	// Send a command which tells the device to periodically publish its speed
				payload_buf[0] = 0x03; // 0x01 for periodically, 0x03 for periodically but only if there is a value change
				time = UPDATE_SPEED_EVERY_MS;
				payload_buf[1] = *(((uint8*)&time));
				payload_buf[2] = *(((uint8*)&time) + 1);
				if (ioHandler->sendCommand(0x44, payload_buf, 3, STATISTICS_COMMAND_TIMEOUT)) {
					init_step++;
					notInterruptableSendAction = true;
				}
				return;
			case 3:	// Send a command which tells the device to periodically publish its force
				payload_buf[0] = 0x03; // 0x01 for periodically, 0x03 for periodically but only if there is a value change
				time = UPDATE_FORCE_EVERY_MS;
				payload_buf[1] = *(((uint8*)&time));
				payload_buf[2] = *(((uint8*)&time) + 1);
				if (ioHandler->sendCommand(0x45, payload_buf, 3, STATISTICS_COMMAND_TIMEOUT)) {
					init_step++;
					notInterruptableSendAction = true;
				}
				return;
			case 4:	// Send a command which tells the device to periodically publish its grasping state
				payload_buf[0] = 0x03; // 0x01 for periodically, 0x03 for periodically but only if there is a value change
				time = UPDATE_GRASPING_STATE_EVERY_MS;
				payload_buf[1] = *(((uint8*)&time));
				payload_buf[2] = *(((uint8*)&time) + 1);
				if (ioHandler->sendCommand(0x41, payload_buf, 3, STATISTICS_COMMAND_TIMEOUT)) {
					init = false;
					notInterruptableSendAction = true;
				}
				return;
			}
		}

		if (sendFastStop) {
			ioHandler->cancelCommand();
			if (ioHandler->sendCommand(0x23, payload_buf, 0, STATISTICS_COMMAND_TIMEOUT)) {
				notInterruptableSendAction = true;
				sendFastStop = false;
			}
			return;
		}

		if (sendStop) {
			ioHandler->cancelCommand();
			if (ioHandler->sendCommand(0x22, payload_buf, 0, STATISTICS_COMMAND_TIMEOUT)) {
				notInterruptableSendAction = true;
				sendStop = false;
			}
			return;
		}
	}

	SchunkWsgDevice* SchunkWsgCanDevice::createDevice(std::string name, RPI::parameter_t parameters) {
		SchunkWsgCanDevice* ret = new SchunkWsgCanDevice(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	void SchunkWsgCanDevice::updateParameters() {

	}

	std::set<std::string> SchunkWsgCanDevice::getMutableParameters() const {
		return std::set<std::string>();
	}

	void SchunkWsgCanDevice::setEStop(bool estop) {
		RTT::os::MutexLock lock(schunkwsgMutex);
		eStop = estop;
		if (eStop) sendFastStop = true;
		else acknowledgeFaststopOrFault();
	}

	RPI::DeviceState SchunkWsgCanDevice::getDeviceState() const {
		return deviceState;
	}

	void SchunkWsgCanDevice::floatByteArrayOfDouble(double value, uint8* destination)
	{
		float tmp = value;
		if (tmp!=value) {
			if (value>0) tmp = FLT_MAX;
			else tmp = FLT_MIN;
		}
		memcpy(destination, &tmp, sizeof(float));
	}

} /* namespace schunkwsg */
