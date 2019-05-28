/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "DirectIO.hpp"

#include <boost/algorithm/string.hpp>
#include <fcntl.h>
#include "../Registry.hpp"

#ifdef WIN32
#define closes closesocket
#else
#define closes close
#endif

using namespace std;

namespace RPI
{

	DirectIO::DirectIO() :
			TaskContext("DirectIO", PreOperational), serversocket(0), port(8081)
	{
		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0, "DirectIOMaster"));
	}

	DirectIO::~DirectIO()
	{
		for (vector<DirectIONetcommHandler*>::iterator it = clients.begin(); it != clients.end(); ++it)
		{
			(*it)->stop();
			(*it)->cleanup();
			delete *it;
		}
		clients.clear();
	}

	bool DirectIO::configureHook()
	{
#ifdef WIN32
		WSADATA WSAData;
		WSAStartup(MAKEWORD(2,0), &WSAData);
#endif

		serversocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (serversocket < 0)
		{
			return false;
		}
		memset(&serversocketaddr, 0, sizeof(serversocketaddr));
		serversocketaddr.sin_family = AF_INET;
		serversocketaddr.sin_port = htons(port);
		serversocketaddr.sin_addr.s_addr = INADDR_ANY;

#ifndef WIN32
		int reuseaddr = 1;
		setsockopt(serversocket, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr));
#endif

		if (::bind(serversocket, (sockaddr*) &serversocketaddr, sizeof(serversocketaddr)) < 0)
		{
			closes(serversocket);
			return false;
		}
		if (listen(serversocket, 2) < 0)
		{
			closes(serversocket);
			return false;
		}
		return true;
	}

	bool DirectIO::startHook()
	{
		return true;
	}

	void DirectIO::updateHook()
	{
		fd_set read_set;
		struct timeval tv;
		int max_fd;
		FD_ZERO(&read_set);
		max_fd = serversocket;
		FD_SET(serversocket, &read_set);
		tv.tv_sec = 0;
		tv.tv_usec = 10 * 1000;
		int retval = select(max_fd + 1, &read_set, NULL, NULL, &tv);
		if (retval < 0)
		{
			//cout << retval << errno << endl;
#ifdef WIN32
			sleep(1);
#endif
		} else
		{
			// accept connection
			if (FD_ISSET(serversocket, &read_set))
			{
				int clientsocket;

				socklen_t len = sizeof(serversocketaddr);
				clientsocket = accept(serversocket, (sockaddr*) &serversocketaddr, &len);
				int flag = 1;
				setsockopt(clientsocket, IPPROTO_TCP, TCP_NODELAY, (char*) &flag, sizeof(int));

				// Client has been connected
				if (clientsocket >= 0)
				{
					DirectIONetcommHandler* handler = new DirectIONetcommHandler(clientsocket);
					handler->start();
					clients.push_back(handler);
				}
			}
		}
		// scan clients for finished threads
		for (vector<DirectIONetcommHandler*>::iterator it = clients.begin(); it != clients.end(); ++it)
		{
			if ((*it)->finished)
			{
				(*it)->cleanup();
				delete *it;
				clients.erase(it);
				break;
			}
		}

		this->trigger();
	}

	void DirectIO::cleanupHook()
	{
		closes(serversocket);
#ifdef WIN32
		WSACleanup();
#endif
	}

	DirectIO *DirectIO::instance = 0;
	DirectIO *DirectIO::getInstance()
	{
		if (!instance)
		{
			instance = new DirectIO();
		}
		return instance;
	}

	void DirectIO::finalize()
	{
		if (instance)
		{
			instance->stop();
			instance->cleanup();
			delete instance;
			instance = 0;
		}
	}

	void DirectIO::setPort(int port)
	{
		this->port = port;
	}

	DirectIONetcommHandler::DirectIONetcommHandler(int clientsocket) :
			TaskContext("DirectIOHandler", Stopped)
	{
		this->clientsocket = clientsocket;
		this->finished = false;

		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0, "DirectIOHandler"));
	}

	DirectIONetcommHandler::~DirectIONetcommHandler()
	{
		if (clientsocket >= 0)
		{
			closes(clientsocket);
		}
	}

	bool DirectIONetcommHandler::configureHook()
	{
		return true;
	}

	bool DirectIONetcommHandler::startHook()
	{
		return true;
	}

	void DirectIONetcommHandler::updateHook()
	{
		fd_set read_set;
		FD_ZERO(&read_set);
		FD_SET(clientsocket, &read_set);
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 1000;
		if (select(clientsocket + 1, &read_set, NULL, NULL, &tv) < 0)
		{
#ifdef WIN32
			sleep(1);
#endif
		} else
		{
			if (FD_ISSET(clientsocket, &read_set))
			{
				char buffer[1024];
				int recvsize;

				recvsize = recv(clientsocket, buffer, 1024, 0);

				// other error
				if (recvsize <= 0)
				{
					stop();
					finished = true;
					return;
				}

				string sbuf(buffer, recvsize);
				//boost::trim(sbuf);

				recvdata += sbuf;

				while (handleInput())
				{
					// first look for updates, then acknowledge or reject command
					auto upd = protocolHandler.updateStatus();
					for (const auto& updcmd : upd)
					{
						string tosend = updcmd->toString() + "#\n";
						send(clientsocket, tosend.c_str(), tosend.size(), 0);
					}

					senddata += "#\n";
					send(clientsocket, senddata.c_str(), senddata.size(), 0);
				}
			}
		}

		auto upd = protocolHandler.updateStatus();
		for (const auto& updcmd : upd)
		{
			string tosend = updcmd->toString() + "#\n";
			send(clientsocket, tosend.c_str(), tosend.size(), 0);
		}

		if (protocolHandler.dofinish)
		{
			stop();
			finished = true;
			return;
		}

		this->trigger();
	}

	bool DirectIONetcommHandler::handleInput()
	{
		const string enddelimiter = "#";
		const int endlength = enddelimiter.length();

		size_t pos = recvdata.find(enddelimiter);
		if (pos != string::npos)
		{
			string extract = recvdata.substr(0, pos + endlength - 1);

			boost::trim(extract);

			senddata = protocolHandler.handleInput(extract)->toString();

			recvdata = recvdata.substr(pos + endlength, recvdata.length() - pos - endlength);

			return true;
		}
		return false;
	}

	void DirectIONetcommHandler::cleanupHook()
	{

	}

	void DirectIONetcommHandler::stopHook()
	{
		if (clientsocket >= 0)
		{
			closes(clientsocket);
			clientsocket = -1;
		}
	}

	bool DirectIONetcommHandler::breakUpdateHook()
	{
		return true;
	}
}
