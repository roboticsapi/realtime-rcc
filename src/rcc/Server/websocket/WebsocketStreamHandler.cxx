/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "WebsocketStreamHandler.hpp"
#include "../mongoose.hpp"
#include <iostream>
#include "SHA1.hpp"
#include "Base64.hpp"

namespace RPI
{
	using namespace std;

	WebsocketStreamHandler::WebsocketStreamHandler()
	{
	}

	WebsocketStreamHandler::~WebsocketStreamHandler()
	{
	}

	void WebsocketStreamHandler::write(mg_connection* conn, int opcode, string data)
	{
		char* buf = new char[data.length() + 10];
		int len = 0;
		buf[0] = (1 << 7) + opcode;
		if (data.length() > 65535)
		{
			buf[1] = 127;
			buf[2] = 0;
			buf[3] = 0;
			buf[4] = 0;
			buf[5] = 0;
			buf[6] = (data.length() >> 24) & 0xff;
			buf[7] = (data.length() >> 16) & 0xff;
			buf[8] = (data.length() >> 8) & 0xff;
			buf[9] = data.length() & 0xff;
			memcpy(&buf[10], data.c_str(), data.length());
			len = data.length() + 10;
		} else if (data.length() > 125)
		{
			buf[1] = 126;
			buf[2] = (data.length() >> 8) & 0xff;
			buf[3] = data.length() & 0xff;
			memcpy(&buf[4], data.c_str(), data.length());
			len = data.length() + 4;
		} else
		{
			buf[1] = data.length();
			memcpy(&buf[2], data.c_str(), data.length());
			len = data.length() + 2;
		}
		mg_write(conn, buf, len);
		delete[] buf;
	}

	bool WebsocketStreamHandler::handleRequest(vector<string> path, string method, string data, string get,
			mg_connection* conn)
	{
		const char* upgrade = mg_get_header(conn, "Upgrade");
		const char* keyp = mg_get_header(conn, "Sec-WebSocket-Key");
		if (upgrade == NULL)
			return false;
		if (keyp == NULL)
			return false;
		if (string(upgrade) != "websocket")
			return false;
		string key = string(keyp);
		string response = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

		CSHA1 sha1;
		char hash[20];
		sha1.Update((const unsigned char*) response.c_str(), response.length());
		sha1.Final();
		sha1.GetHash((unsigned char*) &hash);
		string base64hash = base64::encode((unsigned char*) &hash, 20);

		string answer =
				"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: "
						+ base64hash + "\r\n\r\n";
		mg_write(conn, answer.c_str(), answer.length());
		string request = "";
		unsigned char msg[2];

		WebsocketDIO wsdio;
		wsdio.setConnectionData(conn, this);

		wsdio.start();

		while (true)
		{
			if (mg_read(conn, msg, 2) < 2)
				break;

			// if the webserver is going to shut down, abort this websocket session
			if (HTTPServer::getInstance()->isShutdown())
			{
				RTT::log(RTT::Info) << "Stopping WebsocketDIO channel" << RTT::endlog();
				wsdio.stop();
				return true;
			}

			bool fin = (msg[0] >> 7) & 1;
			//bool rsv1 = (msg[0] >> 6) & 1;
			//bool rsv2 = (msg[0] >> 5) & 1;
			//bool rsv3 = (msg[0] >> 4) & 1;
			int opcode = (msg[0] & 0x0F);
			bool mask = (msg[1] >> 7) & 1;
			int len = msg[1] & 0x7F;
			if (len == 126)
			{
				unsigned char len2[2];
				if (mg_read(conn, &len2, 2) < 2)
					break;
				len = (len2[0] << 8) + len2[1];
			} else if (len == 127)
			{
				unsigned char len8[8];
				if (mg_read(conn, &len8, 8) < 8)
					break;
				len = ((((((((((((((len8[0] << 8) + len8[1]) << 8) + len8[2]) << 8) + len8[3]) << 8) + len8[4]) << 8)
						+ len8[5]) << 8) + len8[6]) << 8) + len8[7]);
			}
			char maskkey[4];
			if (mask)
			{
				if (mg_read(conn, &maskkey, 4) < 4)
					break;
			}
			if (len > 8 * 1024 * 1024)
				return true;
			char* data = new char[len];
			if (mg_read(conn, data, len) < len)
				break;
			if (mask)
			{
				for (int i = 0; i < len; i++)
				{
					data[i] = data[i] ^ maskkey[i % 4];
				}
			}

			switch (opcode)
			{
			case 0: // Continuation
			case 1: // Text
			case 2: // Binary
				request += string(data, len);
				if (fin)
				{
					//this->write(conn, 1, "Echo " + request);
					wsdio.websocketWrite(1, wsdio.handleCommand(request));
					request = "";
				}
				break;

			case 8: // CLOSE
				wsdio.websocketWrite(8, string(data, len));
				delete[] data;
				return true;
			case 9: // PING
				wsdio.websocketWrite(10, string(data, len));
				break;
			case 10: // PONG
				break;
			default:
				break;
			}
			// std::cout << "RAW: " << msg.data.opcode << ": " << string(data, len) << std::endl;

			delete[] data;
		}
		return true;
	}

}
