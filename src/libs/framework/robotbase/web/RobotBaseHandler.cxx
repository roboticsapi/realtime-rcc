/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "RobotBaseHandler.hpp"
#include "../interface/RobotBaseInterface.hpp"

#include <rcc/DeviceInstanceT.hpp>

namespace robotbase
{
	using namespace std;
	using namespace RPI;

	std::string RobotBaseHandler::handleRequest(std::vector<std::string> path, std::string method, std::string data,
			std::string get)
	{
		stringstream ret;
		DeviceInstanceT<RobotBaseInterface> di(path[1]);

		if (di.getDevice() == 0)
			return "";

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/robotbase/robotbase.xsl\" ?>\r\n";
		;
		ret << "<robotbase>\r\n";
		ret << std::fixed;

		double a, b, c;
		KDL::Frame pos = di.getDevice()->getMeasuredBasePosition();
		KDL::Twist vel = di.getDevice()->getMeasuredBaseVelocity();

		pos.M.GetRPY(c, b, a);
		ret << "\t<position x=\"" << pos.p.x() << "\" y=\"" << pos.p.y() << "\" theta=\"" << a << "\" />\r\n";

		ret << "\t<velocity x=\"" << vel.vel.x() << "\" y=\"" << vel.vel.y() << "\" theta=\"" << vel.rot.z() << "\" />\r\n";

		ret << "</robotbase>\r\n";
		return ret.str();
	}
} /* namespace robotbase */
