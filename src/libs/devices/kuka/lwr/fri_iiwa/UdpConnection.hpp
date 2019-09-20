/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef UDPCONNECTION_HPP
#define UDPCONNECTION_HPP

#include "frisdk/friConnectionIf.h"

#include <libs/framework/ethernet/interface/EthernetServerUDPInterface.hpp>
#include <rcc/DeviceInstanceT.hpp>


namespace kuka_iiwa
{
   class UdpConnection : public KUKA::FRI::IConnection
   {
   
   public:
      UdpConnection();
      ~UdpConnection();
      virtual bool open(int port, const char *remoteHost = NULL);
      virtual void close();
      virtual bool isOpen() const;
      virtual int receive(char *buffer, int maxSize);
      virtual bool send(const char* buffer, int size);
   
   private:
      struct sockaddr_in _controllerAddr;
      RPI::DeviceInstanceT<ethernet::EthernetServerUDPInterface> udp;

   };
   
}


#endif // UDPCONNECTION_HPP
