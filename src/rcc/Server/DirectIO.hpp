/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DIRECTIO_HPP_
#define DIRECTIO_HPP_

#ifdef WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
//#define socklen_t int
#define MSG_DONTWAIT 0
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif // WIN32
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>
#include "DIOProtocol.hpp"

namespace RPI
{
	class DirectIONetcommHandler: public RTT::TaskContext
	{
	public:
		DirectIONetcommHandler(int clientsocket);
		virtual ~DirectIONetcommHandler();

		bool configureHook();
		bool startHook();
		void updateHook();
		void cleanupHook();
		void stopHook();
		bool breakUpdateHook();
		bool finished;

	private:
		int clientsocket;
		std::string recvdata;
		std::string senddata;

		bool handleInput();

		DIOProtocol protocolHandler;
	};

	class DirectIO: public RTT::TaskContext
	{
	public:
		DirectIO();
		virtual ~DirectIO();

		bool configureHook();
		bool startHook();
		void updateHook();
		void cleanupHook();

		void setPort(int port);

		static DirectIO* getInstance();
		static void finalize();
	private:
		static DirectIO* instance;
		int serversocket;
		sockaddr_in serversocketaddr;
		std::vector<DirectIONetcommHandler*> clients;

		int port;
	};
}
#endif /* DIRECTIO_HPP_ */
