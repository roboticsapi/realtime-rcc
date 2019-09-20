/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "WebsocketDIO.hpp"
#include "WebsocketStreamHandler.hpp"

namespace RPI
{
	using namespace std;

	WebsocketDIO::WebsocketDIO() :
			RTT::os::Thread(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0, 0, "WebsocketDIO"),
			conn(0), wshandler(0)
	{
		dostop = false;

		HTTPServer::getInstance()->addCyclicHandler(this);
	}

	WebsocketDIO::~WebsocketDIO()
	{
		HTTPServer::getInstance()->removeCyclicHandler(this);

		stop();

		RTT::os::MutexLock lock(diomutex);

		conn = 0;
		wshandler = 0;
	}

	string WebsocketDIO::handleCommand(const string& command)
	{
		string ret = dioprotocol.handleInput(command)->toString();
		updateStatusData();
		return ret;
	}

	void WebsocketDIO::updateStatusData()
	{
		RTT::os::MutexLock lock(diomutex);

		auto ret = dioprotocol.updateStatus();
		if (conn && wshandler)
		{
			for (const auto& updcmd : ret)
			{
				wshandler->write(conn, 1, updcmd->toString());
			}
		}

	}

	void WebsocketDIO::loop()
	{
		while(!dostop) {

			updateStatusData();

			usleep(10000);

		}
	}

	void WebsocketDIO::websocketWrite(int opcode, const string& data)
	{
		RTT::os::MutexLock lock(diomutex);

		if (conn && wshandler)
		{
			wshandler->write(conn, opcode, data);
		}
	}

	void WebsocketDIO::setConnectionData(mg_connection* nconn, WebsocketStreamHandler* nwshandler)
	{
		RTT::os::MutexLock lock(diomutex);

		this->conn = nconn;
		this->wshandler = nwshandler;
	}

	bool WebsocketDIO::initialize()
	{
		return true;
	}

	bool WebsocketDIO::breakLoop()
	{
		dostop = true;
		return true;
	}

	void WebsocketDIO::finalize()
	{

	}

} /* namespace RPI */
