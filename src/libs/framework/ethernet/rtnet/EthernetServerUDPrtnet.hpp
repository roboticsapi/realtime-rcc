/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ETHERNETSERVERUDPRTNET_HPP_
#define ETHERNETSERVERUDPRTNET_HPP_

#include "../interface/EthernetServerUDPInterface.hpp"
#include <rcc/Device.hpp>
#include <string>

namespace ethernet
{

	class EthernetServerUDPrtnet: public EthernetServerUDPInterface, public RPI::Device
	{
	public:
		EthernetServerUDPrtnet(std::string name, RPI::parameter_t parameters);
		virtual ~EthernetServerUDPrtnet();

		static const std::string devicename;

		static EthernetServerUDPrtnet* createDevice(std::string name, RPI::parameter_t parameters);

		// Device Interface
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		void setEStop(bool estop);
		virtual RPI::DeviceState getDeviceState() const;

		virtual bool initialize_socket();
		void close_socket();
		virtual ssize_t recvfrom(void *buffer, size_t length, int flags, struct sockaddr *address,
				socklen_t *address_len);
		virtual ssize_t sendto(const void* message, size_t length, int flags, const struct sockaddr *dest_addr,
				socklen_t dest_len);

	private:
		int udp_socket;
		unsigned int port;
	};

} /* namespace ethernet */

#endif /* ETHERNETSERVERUDPRTNET_HPP_ */
