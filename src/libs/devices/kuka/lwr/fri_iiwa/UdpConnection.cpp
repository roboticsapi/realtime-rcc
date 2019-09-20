/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "UdpConnection.hpp"
#include <cstdio>

namespace kuka_iiwa {
	
	UdpConnection::UdpConnection()
	{
	}

	UdpConnection::~UdpConnection()
	{
		close();
	}

	bool UdpConnection::open(int port, const char *remoteHost)
	{
		if (!udp.fetchInstance(remoteHost))
		{
			printf("fetchInstance remote host to %s FAILED !!!\n", remoteHost);
			return false;
		} else if (!udp->initialize_socket())
		{
			printf("initialize socket remote host to %s FAILED !!!\n", remoteHost);
			return false;
		}

		if (remoteHost)
		{
			printf("preinitialized remote host to %s\n", remoteHost);
		}

		return true;
	}

	void UdpConnection::close()
	{
		if (udp.getDevice())
		{
			udp->close_socket();
		}
	}

	bool UdpConnection::isOpen() const
	{
		return (udp.getDevice() != NULL);
	}

	int UdpConnection::receive(char *buffer, int maxSize)
	{

		if (isOpen())
		{
			socklen_t sockAddrSize;
			sockAddrSize = sizeof(struct sockaddr_in);
			return udp->recvfrom(buffer, maxSize, 0, (struct sockaddr *) &_controllerAddr, (socklen_t *) &sockAddrSize);
		}
		return -1;
	}

	bool UdpConnection::send(const char* buffer, int size)
	{
		if (isOpen())
		{
			int sent = udp->sendto(const_cast<char*>(buffer), size, 0, (struct sockaddr *) &_controllerAddr,
					sizeof(_controllerAddr));

			if (sent == size)
			{
				return true;
			}
		}

		return false;
	}

}