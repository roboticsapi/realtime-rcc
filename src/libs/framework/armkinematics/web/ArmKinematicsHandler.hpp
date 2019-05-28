/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ARMKINEMATICSWEBHANDLER_HPP_
#define ARMKINEMATICSWEBHANDLER_HPP_

#include <rcc/Server/HTTPServer.hpp>

namespace armkinematics
{

	class ArmKinematicsHandler: public RPI::HTTPHandler
	{
	public:
		virtual std::string handleRequest(std::vector<std::string> path, std::string method, std::string data, std::string get);
	};


} /* namespace armkinematics */
#endif /* ARMKINEMATICSWEBHANDLER_HPP_ */
