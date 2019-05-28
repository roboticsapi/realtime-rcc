/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EthernetServerUDPstandard.hpp"

#include <string>
#include <cstring>

#ifdef WIN32
#define close closesocket
#endif

namespace ethernet
{
	const std::string EthernetServerUDPstandard::devicename = "ethernet_udp_standard";

	EthernetServerUDPstandard::EthernetServerUDPstandard(std::string name, RPI::parameter_t parameters) :
			Device(name, parameters)
	{
		udp_socket = -1;
		port = getParameterT<unsigned int>("port", 0);
	}

	EthernetServerUDPstandard::~EthernetServerUDPstandard()
	{
		close_socket();
	}

	EthernetServerUDPstandard* EthernetServerUDPstandard::createDevice(std::string name, RPI::parameter_t parameters)
	{
		return new EthernetServerUDPstandard(name, parameters);
	}

	bool EthernetServerUDPstandard::initialize_socket()
	{
		udp_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
		if (udp_socket < 0)
		{
			return false;
		}

		struct sockaddr_in serversocketaddr;

		memset(&serversocketaddr, 0, sizeof(serversocketaddr));
		serversocketaddr.sin_family = AF_INET;
		serversocketaddr.sin_port = htons(port);
		serversocketaddr.sin_addr.s_addr = INADDR_ANY;

#ifndef WIN32
		int reuseaddr = 1;
		::setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr));
#endif

		if (::bind(udp_socket, (sockaddr*) &serversocketaddr, sizeof(serversocketaddr)) < 0)
		{
			::close(udp_socket);
			udp_socket = -1;
			return false;
		}

		return true;
	}

	void EthernetServerUDPstandard::close_socket()
	{
		if (udp_socket >= 0)
		{
			::close(udp_socket);
			udp_socket = -1;
		}

	}

	ssize_t EthernetServerUDPstandard::recvfrom(void *buffer, size_t length, int flags, struct sockaddr *address,
			socklen_t *address_len)
	{
		if (udp_socket < 0)
			return -1;

		return ::recvfrom(udp_socket, (char*) buffer, length, flags, address, address_len);
	}

	ssize_t EthernetServerUDPstandard::sendto(const void* message, size_t length, int flags,
			const struct sockaddr *dest_addr, socklen_t dest_len)
	{
		if (udp_socket < 0)
			return -1;

		return ::sendto(udp_socket, (char*) message, length, flags, dest_addr, dest_len);
	}

	void EthernetServerUDPstandard::updateParameters()
	{

	}
	std::set<std::string> EthernetServerUDPstandard::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void EthernetServerUDPstandard::setEStop(bool estop)
	{

	}

	RPI::DeviceState EthernetServerUDPstandard::getDeviceState() const
	{
		return RPI::DeviceState::OPERATIONAL;
	}

} /* namespace ethernet */
