/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef WEBSOCKETSTREAMHANDLER_HPP_
#define WEBSOCKETSTREAMHANDLER_HPP_

#include "../HTTPServer.hpp"
#include "WebsocketDIO.hpp"

namespace RPI
{
	class WebsocketStreamHandler: public RPI::HTTPStreamHandler
	{
	public:
		WebsocketStreamHandler();
		virtual ~WebsocketStreamHandler();
		virtual bool handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get,
				mg_connection* conn);
		void write(mg_connection* conn, int opcode, std::string data);
	};

}
#endif /* WEBSOCKETSTREAMHANDLER_HPP_ */
