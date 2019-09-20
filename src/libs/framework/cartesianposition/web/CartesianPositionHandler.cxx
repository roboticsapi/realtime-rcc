/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "CartesianPositionHandler.hpp"
#include "../interface/CartesianPositionInterface.hpp"

#include <rcc/DeviceInstanceT.hpp>

namespace cartesianposition
{
	using namespace std;
	using namespace RPI;

	std::string CartesianPositionHandler::handleRequest(std::vector<std::string> path, std::string method, std::string data,
			std::string get)
	{
		stringstream ret;
		DeviceInstanceT<CartesianPositionInterface> di(path[1]);

		if (di.getDevice() == 0)
			return "";

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/cartesianposition/cartesianposition.xsl\" ?>\r\n";
		ret << "<cartesianposition>\r\n";
		ret << std::fixed;

		double a, b, c;
		KDL::Frame msrpos = di->getMeasuredPosition();
		KDL::Twist msrvel = di->getMeasuredVelocity();
		KDL::Frame cmdpos = di->getCommandedPosition();
		KDL::Twist cmdvel = di->getCommandedVelocity();

		msrpos.M.GetRPY(c, b, a);
		ret << "\t<measured " <<
				"x=\"" << msrpos.p.x() << "\" y=\"" << msrpos.p.y() << "\" z=\""  << msrpos.p.z() << "\" " <<
				"a=\"" << a << "\" b=\"" << b << "\" c=\"" << c << "\" " <<
				"vx=\"" << msrvel.vel.x() << "\" vy=\"" << msrvel.vel.y() << "\" vz=\"" << msrvel.vel.z() << "\" " <<
				"rx=\"" << msrvel.rot.x() << "\" ry=\"" << msrvel.rot.y() << "\" rz=\"" << msrvel.rot.z() << "\" />\r\n";

		cmdpos.M.GetRPY(c, b, a);
		ret << "\t<commanded " <<
				"x=\"" << cmdpos.p.x() << "\" y=\"" << cmdpos.p.y() << "\" z=\""  << cmdpos.p.z() << "\" " <<
				"a=\"" << a << "\" b=\"" << b << "\" c=\"" << c << "\" " <<
				"vx=\"" << cmdvel.vel.x() << "\" vy=\"" << cmdvel.vel.y() << "\" vz=\"" << cmdvel.vel.z() << "\" " <<
				"rx=\"" << cmdvel.rot.x() << "\" ry=\"" << cmdvel.rot.y() << "\" rz=\"" << cmdvel.rot.z() << "\" />\r\n";

		ret << "</cartesianposition>\r\n";
		return ret.str();
	}
} /* namespace robotbase */
