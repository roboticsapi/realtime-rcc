/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EthernetServerUDPrtnet.hpp"

#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <rtdm/rtdm.h>

#include <iostream>

namespace ethernet
{
	const std::string EthernetServerUDPrtnet::devicename = "ethernet_udp_rtnet";

	EthernetServerUDPrtnet::EthernetServerUDPrtnet(std::string name, RPI::parameter_t parameters) :
			Device(name, parameters)
	{
		udp_socket = -1;
		port = getParameterT<unsigned int>("port", 0);
	}

	EthernetServerUDPrtnet::~EthernetServerUDPrtnet()
	{
		close_socket();
	}

	EthernetServerUDPrtnet* EthernetServerUDPrtnet::createDevice(std::string name, RPI::parameter_t parameters)
	{
		return new EthernetServerUDPrtnet(name, parameters);
	}

	bool EthernetServerUDPrtnet::initialize_socket()
	{
		udp_socket = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);
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
//		int reuseaddr = 1;
//		rt_dev_setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr));
#endif

		if (rt_dev_bind(udp_socket, (sockaddr*) &serversocketaddr, sizeof(serversocketaddr)) < 0)
		{
			rt_dev_close(udp_socket);
			udp_socket = -1;
			return false;
		}

		return true;
	}

	void EthernetServerUDPrtnet::close_socket()
	{
		if (udp_socket >= 0)
		{
			rt_dev_close(udp_socket);
			udp_socket = -1;
		}

	}

	ssize_t EthernetServerUDPrtnet::recvfrom(void *buffer, size_t length, int flags, struct sockaddr *address,
			socklen_t *address_len)
	{
		if (udp_socket < 0)
			return -1;

		return rt_dev_call(recvfrom, udp_socket, buffer, length, flags, address, address_len);
		//return rt_dev_recv(udp_socket, buffer, length, flags);
	}

	ssize_t EthernetServerUDPrtnet::sendto(const void* message, size_t length, int flags,
			const struct sockaddr *dest_addr, socklen_t dest_len)
	{
		if (udp_socket < 0)
			return -1;

		return rt_dev_sendto(udp_socket, message, length, flags, dest_addr, dest_len);
	}

	void EthernetServerUDPrtnet::updateParameters()
	{

	}
	std::set<std::string> EthernetServerUDPrtnet::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	void EthernetServerUDPrtnet::setEStop(bool estop)
	{

	}

	RPI::DeviceState EthernetServerUDPrtnet::getDeviceState() const
	{
		return RPI::DeviceState::OPERATIONAL;
	}

} /* namespace ethernet */
