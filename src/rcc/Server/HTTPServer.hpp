/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef HTTPSERVER_HPP_
#define HTTPSERVER_HPP_

//#include "mongoose.hpp"
//#include "unistd.h"
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <rtt/rtt-config.h>
#include <rtt/os/Mutex.hpp>
#include <rtt/TaskContext.hpp>

struct mg_context;
struct mg_connection;

namespace RPI
{
	class RTT_EXPORT HTTPHandler
	{
	public:
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string post, std::string get) = 0;
		virtual ~HTTPHandler();
	protected:
		std::string getVar(const std::string& data, const std::string& name);
		std::map<std::string, std::string> getVars(const std::string& data);

		template<typename T>
		T getVarT(const std::string& data, const std::string& name, const T& defaultvalue = T())
		{
			try
			{
				return boost::lexical_cast<T>(getVar(data, name));
			} catch (boost::bad_lexical_cast&)
			{
				return defaultvalue;
			}
		}

	};

	class RTT_EXPORT HTTPStreamHandler
	{
	public:
		virtual bool handleRequest(std::vector<std::string> path, std::string method, std::string post, std::string get, mg_connection *conn) = 0;
		virtual ~HTTPStreamHandler();
	protected:
		std::string getVar(const std::string& data, const std::string& name);
		std::map<std::string, std::string> getVars(const std::string& data);
	};

	class RTT_EXPORT HTTPServer
	{
	public:
		static HTTPServer *getInstance();
		static void cleanup();
		void addHandler(const std::string & uri, HTTPHandler *handler);
		void addStreamHandler(const std::string & uri, HTTPStreamHandler *handler);
		void removeHandler(const std::string & uri);
		bool handleStreamRequest(std::vector<std::string> path, std::string method, std::string post, std::string get, mg_connection* conn);
		std::string handleRequest(std::vector<std::string> path, std::string method, std::string post, std::string get);
		bool start(std::string port);
		void shutdown();
		bool isShutdown() const;

		void addCyclicHandler(RTT::TaskContext*);
		void removeCyclicHandler(RTT::TaskContext*);
	private:
		std::vector<std::pair<std::vector<std::string>, HTTPStreamHandler*> > streamHandlers;
		std::vector<std::pair<std::vector<std::string>, HTTPHandler*> > handlers;

		RTT::os::Mutex cyclicHandlersMutex;
		std::set<RTT::TaskContext*> cyclicHandlers;

		static HTTPServer *instance;
		mg_context *ctx;
		std::vector<std::string> splitURL(const std::string & url);
		HTTPServer();
		~HTTPServer();

		bool doShutdown;
	};

}

#endif /* HTTPSERVER_HPP_ */
