/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHUNKWSGCANDEVICE_HPP_
#define SCHUNKWSGCANDEVICE_HPP_

#include "../interface/SchunkWsgDevice.hpp"
#include "IO.hpp"
#include <rcc/Device.hpp>
#include <rcc/DeviceInstanceT.hpp>
#include <libs/framework/can/interface/CanDevice.hpp>
#include <libs/framework/can/interface/CanMessage.hpp>
#include <libs/framework/can/interface/CanListener.hpp>

#include <rtt/os/Mutex.hpp>
#include <rtt/TaskContext.hpp>

namespace schunkwsg
{

	typedef unsigned char  uint8;
	typedef unsigned short uint16;
	typedef unsigned int   uint32;

	const std::string dev_schunk_wsg_can = "schunk_wsg_can";

	class SchunkWsgCanDevice: public SchunkWsgDevice, public Callback, public RTT::TaskContext
	{
	public:
		SchunkWsgCanDevice(std::string name, RPI::parameter_t parameters);
		virtual ~SchunkWsgCanDevice();

		bool startHook();
		void updateHook();

		static SchunkWsgDevice* createDevice(std::string name, RPI::parameter_t parameters);
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		RPI::DeviceState getDeviceState() const;

		void setEStop(bool estop);

		bool homing(bool openDirection);
		bool preposition(bool relative, double width, double speed, bool stopOnBlock);
		bool grasp(double width, double speed);
		bool release(double openWidth, double speed);
		bool setAcceleration(double acceleration);
		bool setForceLimit(double force);
		bool setSoftLimits(double inner, double outer);
		bool clearSoftLimits();
		void stopDevice();
		bool isBusy();
		StatusCode getStatusCode();
		double getOpeningWidth();
		double getVelocity();
		double getForce();
		GraspingState getGraspingState();

	private:
		IO* ioHandler;

		// Temporary buffer; only parameters of a WSG command
		uint8 payload_buf[IO::PAYLOAD_BUF_SIZE];


		void triggerInitialization();

		void sendCommand(uint16 message_id, uint8 command_id, uint8* parameters, uint16 param_length);
		void messageReceived(uint8 command_id, StatusCode status_code, const uint8* payload, uint16 size);

		void floatByteArrayOfDouble(double value, uint8* destination);

		bool canSend();
		bool acknowledgeFaststopOrFault();

		// For debugging
		void print_canmessage(const can::CanMessage &msg);

		RPI::DeviceInstanceT<can::CanDevice> canBus;
		RTT::os::Mutex schunkwsgMutex;
		RPI::DeviceState deviceState;

		static const uint16 UPDATE_OPENING_WIDTH_EVERY_MS = 50;
		static const uint16 UPDATE_SPEED_EVERY_MS = 50;
		static const uint16 UPDATE_FORCE_EVERY_MS = 50;
		static const uint16 UPDATE_GRASPING_STATE_EVERY_MS = 50;

		static const int MOVE_COMMAND_TIMEOUT = 20;
		static const int STATISTICS_COMMAND_TIMEOUT = 1;

		// allows to send initial packets to the device when this driver starts
		bool init;
		int init_step;

		StatusCode status_code;
		bool busy;
		bool eStop;

		// true, if a system intern command is currently being sent (initial commands, stop, faststop, etc). User commands such as homing or grasp have to wait.
		bool notInterruptableSendAction;

		// If true, the next command sent will be a stop command (fast stop commands are handled with higher priority)
		bool sendStop;

		// If true, the next command sent will be a fast stop command
		bool sendFastStop;

		double currentOpeningWidth;
		double currentVelocity;
		double currentForce;
		GraspingState currentGraspingState;
	};

} /* namespace schunkwsg */
#endif /* SCHUNKWSGCANDEVICE_HPP_ */
