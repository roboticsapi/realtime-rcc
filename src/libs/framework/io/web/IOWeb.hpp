/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef IOWEB_HPP_
#define IOWEB_HPP_

#include <rcc/Server/HTTPServer.hpp>

namespace IO
{

	class RTT_EXPORT IOWeb: public RPI::HTTPHandler
	{
	public:
		IOWeb();
		virtual ~IOWeb();
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};

} /* namespace IO */
#endif /* IOWEB_HPP_ */
