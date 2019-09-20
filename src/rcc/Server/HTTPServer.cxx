/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "HTTPServer.hpp"
#include "../Registry.hpp"
#include "mongoose.hpp"

#ifdef REALTIME
#define MG_MAX_THREADS "10"
#else
#define MG_MAX_THREADS "10"
#endif

using namespace std;

namespace RPI
{

	HTTPHandler::~HTTPHandler()
	{

	}

	HTTPStreamHandler::~HTTPStreamHandler()
	{

	}

	string HTTPHandler::getVar(const string& data, const string& name)
	{
		if(data.length() == 0)
			return "";
		char* result = new char[data.length()];
		int len = mg_get_var(data.c_str(), data.length(), name.c_str(), result, data.length());
		if (len <= 0)
		{
			delete[] result;
			return "";
		} else
		{
			string ret = string(result, len);
			delete[] result;
			return ret;
		}
	}

	map<string, string> HTTPHandler::getVars(const string& data)
	{
		map<string, string> ret;
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

		boost::char_separator<char> sep("&");
		tokenizer tokens(data, sep);
		for(tokenizer::iterator it = tokens.begin(); it != tokens.end(); ++it)
		{
			size_t delpos = it->find("=");
			if(delpos != it->npos) {
				string varname = it->substr(0, delpos);
				ret[varname] = getVar(data, varname);
			}
		}
		return ret;
	}

	static void HTTPServer_show_404(struct mg_connection *conn, const struct mg_request_info *request_info)
	{
		string uri = request_info->uri;
		if (uri == "/")
		{
			mg_printf(conn, "%s", "HTTP/1.1 500 Internal Server Error\r\n");
			mg_printf(conn, "%s", "Content-Type: text/html\r\n\r\n");
			mg_printf(
					conn,
					"%s",
					"<html><head><title>500 Internal Server Error</title></head><body><h1>Internal Server Error</h1>Orocos RCC start page not found. Do you use the correct working directory?</body></html>");
			return;
		}
		if (uri[uri.length() - 1] != '/')
		{
			uri += "/";
		} else
		{
			for (int i = uri.length() - 2; i >= 0; i--)
				if (uri[i] == '/')
				{
					uri = uri.substr(0, i);
					break;
				}
		}

		mg_printf(conn, "%s", "HTTP/1.1 307 Temporary Redirect\r\n");
		mg_printf(conn, "%s%s%s", "Location: ", uri.c_str(), "\r\n");
		mg_printf(conn, "%s", "Content-Type: text/html\r\n\r\n");
		mg_printf(conn, "%s",
				"<html><head><title>301 Found</title></head><body><h1>301 Found</h1>The requested file might have moved.</body></html>");
	}

	static void* HTTPServer_handle(mg_event event, mg_connection *conn, const struct mg_request_info *request_info)
	{
		if (event == MG_HTTP_ERROR)
		{
			if (request_info->status_code == 404)
			{
				HTTPServer_show_404(conn, request_info);
				return conn;
			}
		} else if (event == MG_NEW_REQUEST)
		{
			vector<string> path;
			char ch;
			int i = 1;
			string word = "";
			while ((ch = request_info->uri[i++]))
			{
				if (ch == '/')
				{
					path.push_back(word);
					word = "";
				} else
				{
					word += ch;
				}
			}
			path.push_back(word);

			stringstream sdata;
			if (string(request_info->request_method) == "POST")
			{
				char* rawdata = new char[4097];
				int read;
				while ((read = mg_read(conn, rawdata, 4096)) != 0)
				{
					sdata << string(rawdata, read);
				}
				delete[] rawdata;
			}
			string data = sdata.str();

			if(HTTPServer::getInstance()->handleStreamRequest(path, request_info->request_method, data,
					request_info->query_string == NULL ? "" : string(request_info->query_string), conn))
				return conn;


			string result = HTTPServer::getInstance()->handleRequest(path, request_info->request_method, data,
					request_info->query_string == NULL ? "" : string(request_info->query_string));
			if (result != "")
			{
				mg_write(conn, result.c_str(), result.length());
				return conn;
			}
		}
		return NULL;
	}


	bool HTTPServer::handleStreamRequest(vector<string> path, string method, string data, string get, mg_connection* conn)
	{

		vector<pair<vector<string> , HTTPStreamHandler*> > handlers = HTTPServer::getInstance()->streamHandlers;

		for (vector<pair<vector<string> , HTTPStreamHandler*> >::iterator it = handlers.begin(); it != handlers.end(); ++it)
		{
			vector<string> uri = it->first;
			if (path.size() == uri.size())
			{
				bool ok = true;
				for (unsigned int i = 0; i < path.size() && i < uri.size(); i++)
				{
					if (uri[i] != "*" && uri[i] != path[i])
					{
						ok = false;
						break;
					}
				}

				if (ok)
				{
					if(it->second->handleRequest(path, method, data, get, conn))
						return true;
				}
			}
		}
		return false;
	}


	string HTTPServer::handleRequest(vector<string> path, string method, string data, string get)
	{

		//vector<pair<vector<string> , HTTPHandler*> > handlers = HTTPServer::getInstance()->handlers;

		for (const auto& handler: handlers)
		{
			auto& uri = handler.first;

			if (path.size() == uri.size())
			{
				bool ok = true;
				for (unsigned int i = 0; i < path.size() && i < uri.size(); i++)
				{
					if (uri[i] != "*" && uri[i] != path[i])
					{
						ok = false;
						break;
					}
				}

				if (ok)
				{
					string result = handler.second->handleRequest(path, method, data, get);
					if (result != "")
					{
						return result;
					}
				}
			}
		}

		return "";
	}

	HTTPServer* HTTPServer::instance = 0;

	HTTPServer* HTTPServer::getInstance()
	{
		if (instance == 0)
			instance = new HTTPServer();
		return instance;
	}
	void HTTPServer::cleanup()
	{
		delete instance;
		instance = 0;
	}
	HTTPServer::HTTPServer() : ctx(0)
	{
		doShutdown = false;
	}

	vector<string> HTTPServer::splitURL(const string & url)
    {
        vector<string> path;
        char ch;
        int i = 1;
        string word = "";
        const char *uri = url.c_str();
        while((ch = uri[i++])){
            if(ch == '/'){
                path.push_back(word);
                word = "";
            }else{
                word += ch;
            }
        }

        path.push_back(word);
        return path;
    }

    void HTTPServer::addHandler(const string & url, HTTPHandler *handler)
    {
        vector<string> path = splitURL(url);
		handlers.push_back(pair<vector<string> , HTTPHandler*> (path, handler));
	}

    void HTTPServer::addStreamHandler(const string & url, HTTPStreamHandler *handler)
    {
        vector<string> path = splitURL(url);
		streamHandlers.push_back(pair<vector<string> , HTTPStreamHandler*> (path, handler));
	}

	void HTTPServer::removeHandler(const string& url)
	{
		vector<string> path = splitURL(url);
		for (vector<pair<vector<string>, HTTPHandler*> >::iterator it = handlers.begin(); it != handlers.end(); ++it)
		{
			if (it->first == path)
			{
				handlers.erase(it);
				break;
			}
		}
	}

	HTTPServer::~HTTPServer()
	{
		if (ctx)
			mg_stop(ctx);
		ctx = 0;
	}

	void HTTPServer::shutdown()
	{
		doShutdown = true;

		// stop all registered cyclic handlers
		{
			RTT::os::MutexLock cycHand(cyclicHandlersMutex);

			for(const auto& handler: cyclicHandlers)
				handler->stop();
		}

		if (ctx)
			mg_shutdown(ctx);
	}
	bool HTTPServer::start(string port)
	{
		//   const char *options[] = {
		//     "document_root", "/var/www",
		//     "listening_ports", "80,443s",
		//     NULL
		//   };


		const char *options[] =
		{ "p", port.c_str(), "r", "web", "m", "xsl=text/xml,xml=text/xml", "t", MG_MAX_THREADS, NULL };

		ctx = mg_start(&HTTPServer_handle, NULL, (const char**) options);

		return (ctx != NULL);

	}

	bool HTTPServer::isShutdown() const
	{
		return doShutdown;
	}

	void HTTPServer::addCyclicHandler(RTT::os::Thread* task)
	{
		RTT::os::MutexLock cychand(cyclicHandlersMutex);
		cyclicHandlers.insert(task);
	}

	void HTTPServer::removeCyclicHandler(RTT::os::Thread* task)
	{
		RTT::os::MutexLock cychand(cyclicHandlersMutex);
		cyclicHandlers.erase(task);
	}
}
