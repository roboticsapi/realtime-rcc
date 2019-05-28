/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "RobotArmHandler.hpp"
#include "../interface/RobotArmInterface.hpp"
#include <rcc/DeviceFactory.hpp>
#include <rcc/DeviceInstanceT.hpp>

namespace robotarm
{
	using namespace std;
	using namespace RPI;

	std::string RobotArmHandler::handleRequest(std::vector<std::string> path, std::string method, std::string data,
			std::string get)
	{
		stringstream ret;
		DeviceInstanceT<RobotArmInterface> di(path[1]);

		if (di.getDevice() == 0)
			return "";

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/robotarm/robotarm.xsl\" ?>\r\n";;
		ret << "<robotarm>\r\n";
		ret << std::fixed;

		for (int i = 0; i < di.getDevice()->getJointCount(); ++i)
		{
			ret << "<joint nr=\"" << i << "\" ";
			ret << "measured=\"" << di.getDevice()->getMeasuredJointPosition(i) << "\" ";
			ret << "commanded=\"" <<  di.getDevice()->getCommandedJointPosition(i) << "\" />\r\n";
		}

		ret << "</robotarm>\r\n";
		return ret.str();
	}

} /* namespace robotarm */
