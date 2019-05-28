/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "../DIOProtocol.hpp"
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>
#include <rtt/os/Mutex.hpp>
#include "../mongoose.hpp"

#ifndef WEBSOCKETDIO_HPP_
#define WEBSOCKETDIO_HPP_

namespace RPI
{
	class WebsocketStreamHandler;

	class WebsocketDIO: public RTT::TaskContext
	{
	public:
		WebsocketDIO();
		virtual ~WebsocketDIO();

		std::string handleCommand(const std::string&);

		void websocketWrite(int opcode, const std::string& data);
		void setConnectionData(mg_connection*, WebsocketStreamHandler*);

		void updateStatusData();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	private:
		DIOProtocol dioprotocol;
		bool dostop;

		mg_connection* conn;
		WebsocketStreamHandler* wshandler;

		RTT::os::Mutex diomutex;
	};

} /* namespace RPI */
#endif /* WEBSOCKETDIO_HPP_ */
