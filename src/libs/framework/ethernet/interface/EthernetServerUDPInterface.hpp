/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ETHERNETSERVERINTERFACE_HPP_
#define ETHERNETSERVERINTERFACE_HPP_

#ifdef WIN32
// WIN 32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#include <ws2tcpip.h>
#define ssize_t SSIZE_T
#define MSG_DONTWAIT 0
#else
// Linux
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif // WIN32

namespace ethernet
{
	class EthernetServerUDPInterface
	{
	public:
		EthernetServerUDPInterface()
		{
		}

		virtual ~EthernetServerUDPInterface()
		{
		}


		virtual bool initialize_socket() = 0;
		virtual void close_socket() = 0;
		virtual ssize_t recvfrom(void *buffer, size_t length, int flags, struct sockaddr *address, socklen_t *address_len) = 0;
		virtual ssize_t sendto(const void* message, size_t length, int flags, const struct sockaddr *dest_addr, socklen_t dest_len) = 0;

	};
}

#endif /* ETHERNETSERVERINTERFACE_HPP_ */
