/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef IO_HPP_
#define IO_HPP_

#include "Callback.hpp"
#include <libs/framework/can/interface/CanDevice.hpp>
#include <libs/framework/can/interface/CanListener.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/os/Mutex.hpp>

namespace schunkwsg
{
	typedef unsigned char  uint8;
	typedef unsigned short uint16;
	typedef unsigned int uint32;

	class IO : can::CanListener
	{
	public:
		IO(can::CanDevice* canDevice, Callback* callBack, uint8 id_write, uint8 id_read);
		virtual ~IO();

		void dataReceived(const can::CanMessage* message);

		// Enqueues new data to be sent. Returns false if there is an irreparable error such as a malformed message_id or data too big for the static buffer.
		bool sendCommand(uint8 command_id, uint8* parameters, uint16 param_length, double timeout);
		bool isTimeout();
		bool isSuccessful();
		bool isError();
		bool isCanceled();
		bool isTerminated();
		void cancelCommand();
		StatusCode getStatusCode();

		static const uint8 MAX_CANMESSAGES_PER_COMMAND = 8;
		static const int PAYLOAD_BUF_SIZE = (MAX_CANMESSAGES_PER_COMMAND * 12)-8;

		static const int WRITE_TIMEOUT_SECONDS = 1;
		static const int COMMAND_BUF_SIZE = MAX_CANMESSAGES_PER_COMMAND * 12;
		static const int RESPONSE_BUF_SIZE = COMMAND_BUF_SIZE;

	private:
		RTT::os::Mutex schunkwsgIOMutex;

		uint8 id_write;
		uint8 id_read;

		enum State {WRITING, LISTENING, TIMEOUT, SUCCESSFUL, ERROR, CANCELED};

		can::CanDevice* canDevice;

		Callback* callBack;
		double timeout;
		State state;
		RTT::os::TimeService::nsecs timestampOfWriteStart;
		bool cancelAfterWriting;
		StatusCode statusCode;

		// The currently active command id
		uint8 running_comand_id;

		// Buffer for can messages to be sent
		can::CanMessage can_messages[MAX_CANMESSAGES_PER_COMMAND];

		// Size of the can message buffer
		uint8 number_of_messages;

		// Temporary buffer; complete WSG command (maybe bigger than a CAN message)
		uint8 command_buf[COMMAND_BUF_SIZE];

		// Reading buffer for a complete WSG response (maybe bigger than a CAN message)
		uint8 reader_buf[RESPONSE_BUF_SIZE];

		// Size of the reading buffer
		uint8 reader_buf_size;

		// Remaining data size which is expected to be received
		uint8 reader_buf_expected_size;

		void messageReceived(uint8 command_id, StatusCode status_code, const uint8* payload, uint16 size);

		void createMessages(uint8 command_id, uint8* parameters, uint16 param_length);
		void doCyclicJob();
		bool flushSendBuffer();
	};

} /* namespace schunkwsg */
#endif /* IO_HPP_ */
