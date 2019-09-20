/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "CommunicationLogger.hpp"
#include <algorithm>
#include <string.h>

#include <iostream>
#include <fstream>

namespace communicationlogger
{

	static CommunicationLogger* instance;

	CommunicationLogger* CommunicationLogger::getInstance() {
		if (!instance) {
			enterCriticalSection();
			if (!instance) {
				instance = new CommunicationLogger();
			}
			leaveCriticalSection();
		}
		return instance;
	}

	CommunicationLogger::CommunicationLogger()
	{
		buf_start_pos = 0;
		buf_write_pos = 0;
		stopAfterMessages = -1;
		id_read = -1;
	}

	CommunicationLogger::~CommunicationLogger()
	{
		// TODO Auto-generated destructor stub
	}

	void CommunicationLogger::log(const can::CanMessage* message, bool received)
	{
		if (stopAfterMessages==0) return;

		enterCriticalSection();

		time_t sec_timestamp;
		struct timeval timestamp;
		gettimeofday( &timestamp, NULL );
		sec_timestamp = timestamp.tv_sec;

		tm *nun;
		nun = localtime(&sec_timestamp);

		ProtocolEntry* pEntry = &debug_buffer[buf_write_pos];
		buf_write_pos = (buf_write_pos + 1) % buf_size;

		// Buffer full? => Overwrite the first entry
		if (buf_write_pos==buf_start_pos) buf_start_pos = (buf_start_pos + 1) % buf_size;

		pEntry->received = received;
		pEntry->error = 0;
		pEntry->hour = nun->tm_hour;
		pEntry->min = nun->tm_min;
		pEntry->sec = nun->tm_sec;
		pEntry->usec = timestamp.tv_usec;
		pEntry->message = *message;

		if (stopAfterMessages!=-1) {
			stopAfterMessages --;
		}

		// Fehlererkennung

		if (received) {
			if (id_read==-1) {
				id_read = message->message_id;
			}
			if (message->message_id!=id_read) {
				pEntry->error = 1;
				if (stopAfterMessages==-1) stopAfterMessages = countdown;
			}

			// New data?
			if (message->data[0]==0xAA && message->data[1]==0xAA && message->data[2]==0xAA) {
				if (reader_buf_expected_size!=0) {
					pEntry->error = 2;
					if (stopAfterMessages==-1) stopAfterMessages = countdown;
				}

				// clear read buffer
				uint16 payload_len = message->data[4] + (message->data[5] * 0x100);
				reader_buf_size = 0;
				reader_buf_expected_size = 8 + payload_len;
			}

			// received message is too long?
			if (message->length > reader_buf_expected_size) {
				pEntry->error = 3;
				if (stopAfterMessages==-1) stopAfterMessages = countdown;
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
				unsigned short chksum = checksum_update_crc16(reader_buf, reader_buf_size, 0xFFFF);
				if (chksum!=0) {
					pEntry->error = 4;
					if (stopAfterMessages==-1) stopAfterMessages = countdown;
				}

				reader_buf_size = 0;
			}
		}

		leaveCriticalSection();

		if (stopAfterMessages==0) {
			// Schreiben
			std::ofstream myfile;
			myfile.open ("/home/ludwig/commlog.txt");
			while (buf_write_pos!=buf_start_pos) {
				ProtocolEntry* entry = &debug_buffer[buf_start_pos];
				if (entry->error==0) myfile << "OK  ";
				else myfile << "ERR" << entry->error;
				myfile << "	";
				myfile << (entry->received ? "in " : "out");
				myfile << "	";
				myfile << entry->hour << ":" << entry->min << ":" << entry->sec << ":" << entry->usec << "	";
				myfile << "ID: " << entry->message.message_id << ", ";
				myfile << "RTR: " << (int)entry->message.rtr << ", ";
				myfile << "DLC: " << (int)entry->message.length << ", ";
				myfile << "DATA: ";
				for (int i=0; i<entry->message.length; i++) {
					myfile << (i==0 ? "" : " ") << (int)entry->message.data[i];
				}
				myfile << "\n";

				buf_start_pos = (buf_start_pos + 1) % buf_size;
			}
			myfile.close();
			std::cout << "File written" << std::endl;
		}
	}

} /* namespace communicationlogger */
