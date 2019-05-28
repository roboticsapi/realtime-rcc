/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "IO.hpp"
#include "CRC.hpp"

#include <rtt/Logger.hpp>

namespace schunkwsg
{

	IO::IO(can::CanDevice* _canDevice, Callback* _callBack, uint8 _id_write, uint8 _id_read)
	{
		if (_id_read > 0x7FF) {
			RTT::log(RTT::Error) << "Can read id (" << _id_read << ") is greater than 11 bits" << RTT::endlog();
		}
		if (_id_write > 0x7FF) {
			RTT::log(RTT::Error) << "Can write id (" << _id_write << ") is greater than 11 bits" << RTT::endlog();
		}

		canDevice = _canDevice;
		callBack = _callBack;
		timeout = 0;
		state = SUCCESSFUL;
		statusCode = E_SUCCESS;
		id_write = _id_write;
		id_read = _id_read;

		running_comand_id = 0;
		number_of_messages = 0;
		cancelAfterWriting = false;

		reader_buf_size = 0;
		reader_buf_expected_size = 0;

		timestampOfWriteStart = RTT::os::TimeService::Instance()->getNSecs();

		canDevice->addCanListener(this, id_read);
	}

	IO::~IO()
	{
		canDevice->removeCanListener(this, id_read);
	}

	void IO::dataReceived(const can::CanMessage* message)
	{
		if (message->message_id!=id_read) return;

		// New data?
		if (message->data[0]==0xAA && message->data[1]==0xAA && message->data[2]==0xAA) {
			if (reader_buf_expected_size!=0) {
				// Error: Previous command was not finished correctly.
				RTT::log(RTT::Error) << "Incomplete message received. I'm going to ignore this message..." << RTT::endlog();
			}

			// clear read buffer
			uint16 payload_len = message->data[4] + (message->data[5] * 0x100);
			reader_buf_size = 0;
			reader_buf_expected_size = 8 + payload_len;
		}

		// received message is too long?
		if (message->length > reader_buf_expected_size) {
			RTT::log(RTT::Error) << "Received message is longer than expected (what the header info says). I'm going to ignore the remaining data bytes..." << RTT::endlog();
		}

		// append to read buffer
		uint8 bytes_to_read = std::min(message->length, reader_buf_expected_size);
		memcpy(reader_buf+reader_buf_size, message->data, bytes_to_read);
		reader_buf_size += bytes_to_read;
		reader_buf_expected_size -= bytes_to_read;

		// received all expected data?
		if (reader_buf_expected_size==0) {
			uint8 command_id = reader_buf[3];
			uint16 status_code = reader_buf[6] + (reader_buf[7] * 0x100);
			const uint8* params = &(reader_buf[8]);
			uint8 param_len = reader_buf[4] + (reader_buf[5] * 0x100);
			uint16 crc = reader_buf[reader_buf_size-2] + (reader_buf[reader_buf_size-1] * 0x100);

			// crc correct?
			unsigned short chksum = crc::checksum_update_crc16(reader_buf, reader_buf_size, 0xFFFF);
			if (chksum!=0) {
				RTT::log(RTT::Error) << "Message with wrong checksum received. I'm going to ignore this message..." << RTT::endlog();
			}
			else {
				messageReceived(command_id, (StatusCode) status_code, params, param_len-2);
			}
			reader_buf_size = 0;
		}
	}

	void IO::messageReceived(uint8 command_id, StatusCode _status_code, const uint8* payload, uint16 size)
	{
		{
			RTT::os::MutexLock lock(schunkwsgIOMutex);

			if (state==LISTENING && command_id==running_comand_id) {
				if (_status_code==E_CMD_PENDING) {}
				else if (_status_code==E_SUCCESS) {
					state = SUCCESSFUL;
					statusCode = E_SUCCESS;
				}
				else {
					state = ERROR;
					statusCode = _status_code;
				}
			}
		}
		callBack->messageReceived(command_id, (StatusCode) _status_code, payload, size);
	}

	bool IO::sendCommand(uint8 command_id, uint8* parameters, uint16 param_length, double _timeout)
	{
		RTT::os::MutexLock lock(schunkwsgIOMutex);

		if (param_length > PAYLOAD_BUF_SIZE) {
			RTT::log(RTT::Error) << "Can message data size (" << param_length << ") is too big for the buffer used. Job aborted." << RTT::endlog();
			return false;
		}
		if (state==WRITING || state==LISTENING) {
			return false;
		}

		timestampOfWriteStart = RTT::os::TimeService::Instance()->getNSecs();
		timeout = _timeout;
		state = WRITING;
		statusCode = E_SUCCESS;
		running_comand_id = command_id;
		createMessages(command_id, parameters, param_length);
		doCyclicJob();
		return true;
	}

	bool IO::isTimeout()
	{
		RTT::os::MutexLock lock(schunkwsgIOMutex);

		doCyclicJob();
		return state==TIMEOUT;
	}

	bool IO::isSuccessful()
	{
		RTT::os::MutexLock lock(schunkwsgIOMutex);

		doCyclicJob();
		return state==SUCCESSFUL;
	}

	bool IO::isError()
	{
		RTT::os::MutexLock lock(schunkwsgIOMutex);

		doCyclicJob();
		return state==ERROR;
	}

	bool IO::isCanceled()
	{
		RTT::os::MutexLock lock(schunkwsgIOMutex);

		doCyclicJob();
		return state==CANCELED;
	}

	bool IO::isTerminated()
	{
		RTT::os::MutexLock lock(schunkwsgIOMutex);

		doCyclicJob();
		return state==SUCCESSFUL || state==TIMEOUT || state==ERROR || state==CANCELED;
	}

	void IO::cancelCommand()
	{
		RTT::os::MutexLock lock(schunkwsgIOMutex);

		if (state!=SUCCESSFUL && state!=WRITING) {
			state = CANCELED;
			statusCode = E_CMD_ABORTED;
		}
		else if (state==WRITING) cancelAfterWriting = true;
	}

	StatusCode IO::getStatusCode()
	{
		return statusCode;
	}

	void IO::doCyclicJob()
	{
		RTT::os::TimeService::Seconds elapsed = RTT::os::TimeService::Instance()->getNSecs(timestampOfWriteStart) / 1e9;

		if (state==WRITING) {
			if (elapsed >= WRITE_TIMEOUT_SECONDS) {
				state = TIMEOUT;
				statusCode = E_TIMEOUT;
				return;
			}
			if (flushSendBuffer()) {
				if (cancelAfterWriting) {
					state = CANCELED;
					statusCode = E_CMD_ABORTED;
					cancelAfterWriting = false;
				}
				else {
					state = LISTENING;
					statusCode = E_SUCCESS;
				}
			}
		}

		if (state==WRITING || state==LISTENING) {
			if (elapsed >= timeout) {
				state = TIMEOUT;
				statusCode = E_TIMEOUT;
				return;
			}
		}
	}

	void IO::createMessages(uint8 command_id, uint8* parameters, uint16 param_length)
	{
		command_buf[0] = 0xAA;
		command_buf[1] = 0xAA;
		command_buf[2] = 0xAA;
		command_buf[3] = command_id;
		command_buf[4] = param_length & 0xFF; // low
		command_buf[5] = (param_length >> 8) & 0xFF; // high
		memcpy(&(command_buf[6]), parameters, param_length);
		unsigned short chksum = crc::checksum_update_crc16(command_buf, 6 + param_length, 0xFFFF);
		command_buf[6 + param_length] = chksum & 0xFF; // low
		command_buf[7 + param_length] = (chksum >> 8) & 0xFF; // high

		uint32 length = 8 + param_length;
		uint8 i = 0;
		while (length > 0)
		{
			can::CanMessage &msg = can_messages[i];
			msg.message_id = id_write;
			msg.rtr = 0;
			msg.length = can::CanMessageDataSize > length ? length : can::CanMessageDataSize;
			memcpy(msg.data, &(command_buf[can::CanMessageDataSize*i]), msg.length);
			length -= msg.length;
			i ++;
		}
		number_of_messages = i;
	}

	/**
	* Sends the messages contained in the message buffer. Returns true if the buffer could be completely sent and is empty, otherwise false.
	*/
	bool IO::flushSendBuffer() {
		while (number_of_messages>0 && canDevice->sendCanMessage(&(can_messages[0]))) {
			for (int i=0; i<MAX_CANMESSAGES_PER_COMMAND-1; i++) {
				can_messages[i] = can_messages[i+1];
			}
			number_of_messages --;
		}
		if (number_of_messages==0) return true;
		return false;
	}

} /* namespace schunkwsg */
