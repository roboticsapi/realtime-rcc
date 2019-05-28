/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef HTTPHANDLERS_HPP_
#define HTTPHANDLERS_HPP_

#include "HTTPServer.hpp"
#include "rtt/os/Mutex.hpp"

namespace RPI
{

	class ExtensionsHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class DevicesHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class RCCDeviceHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class DeviceParameterHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};


	class SessionsHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class SessionHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};


	class NetsHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class NetHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class NetXmlHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class NetDioHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class NetDebugHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class NetDebugStreamHandler : public HTTPStreamHandler {
		virtual bool handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get, mg_connection* conn);
	};

	class NetStatsHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class ModulesDetailsHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class EvalHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class TypesHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class TypeHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class TypeSourceHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};


	class ModulesHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class ModuleHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	private:
		RTT::os::Mutex mutex;
	};

	class ModuleSourceHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class ModuleJavaSourceHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

	class LogHandler : public HTTPHandler {
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};
}

#endif
