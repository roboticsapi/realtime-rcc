/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "web.hpp"
#include <rcc/DeviceInstanceT.hpp>

namespace kuka_youbot
{
	using namespace RPI;
	using namespace std;

	string ybStatsHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;
		DeviceInstanceT<youBotArm> di(path[1]);

		if (di.getDevice() == 0)
			return "";

		if (getVar(get, "p") != "")
			di.getDevice()->setControllerPositionGainConstant(0,atof(getVar(get, "p").c_str()));

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/kuka/youbot/stats.xsl\" ?>\r\n"
				<< "<controller name=\"" << path[1] << "\">";

		std::string f1, f2, f3, f4, f5;
		di.getDevice()->getJointFirmware(f1, f2, f3, f4, f5);
		ret << "	<firmwarejoints j1=\"" << f1 << "\" j2=\"" << f2 << "\" j3=\"" << f3 << "\" j4=\"" << f4 << "\" j5=\""
				<< f5 << "\" />\r\n";

		int ParamNumber[31] =
		{ 151, 152, 253, 239, 240, 238, 136, 25, 26, 27, 28, 30, 245, 254, 250, 209, 251, 165, 177, 249, 241, 244, 14,
				15, 159, 160, 247, 167, 242, 243, 11 };
		for (int i = 0; i < 31; i++)
		{

			int w1, w2, w3, w4, w5;
			di.getDevice()->getJointParameter(w1, w2, w3, w4, w5, ParamNumber[i]);

			ret << "	<p" << ParamNumber[i] << " j1=\"" << w1 << "\" j2=\"" << w2 << "\" j3=\"" << w3 << "\" j4=\"" << w4
					<< "\" j5=\"" << w5 << "\" />\r\n";

		}

		ret << "</controller>\r\n";
		return ret.str();
	}

	string ybControllerHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;
		DeviceInstanceT<youBotArm> di(path[1]);

		if (di.getDevice() == 0)
			return "";

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/kuka/youbot/arm.xsl\" ?>\r\n"
				<< "<controller name=\"" << path[2] << "\">\r\n";

		//ret << "	<gripper value=\"" << di.getDevice()->getGripperPosition() << "\"/>\n";

		float j1, j2, j3, j4, j5;
		j1 = di->getMeasuredJointPosition(0);
		j2 = di->getMeasuredJointPosition(1);
		j3 = di->getMeasuredJointPosition(2);
		j4 = di->getMeasuredJointPosition(3);
		j5 = di->getMeasuredJointPosition(4);
		float cj1, cj2, cj3, cj4, cj5;
		cj1 = di->getCommandedJointPosition(0);
		cj2 = di->getCommandedJointPosition(1);
		cj3 = di->getCommandedJointPosition(2);
		cj4 = di->getCommandedJointPosition(3);
		cj5 = di->getCommandedJointPosition(4);

		yb_kin calc;
		KDL::Frame frame;
		calc.ybKin(j1, j2, j3, j4, j5, frame);

		double r, p, y;
		frame.M.GetRPY(r, p, y);
		ret << "	<frame x=\"" << frame.p.x() << "\" y=\"" << frame.p.y() << "\" z=\"" << frame.p.z() << "\" a=\"" << y
				<< "\" b=\"" << p << "\" c=\"" << r << "\" />\r\n";

		ret << "	<joints j1=\"" << j1 << "\" j2=\"" << j2 << "\" j3=\"" << j3 << "\" j4=\"" << j4 << "\" j5=\"" << j5
				<< "\" />\r\n";

		ret << "	<cmdjoints j1=\"" << cj1 << "\" j2=\"" << cj2 << "\" j3=\"" << cj3 << "\" j4=\"" << cj4 << "\" j5=\""
				<< cj5 << "\" />\r\n";

		calc.ybKin(cj1, cj2, cj3, cj4, cj5, frame);
		ret << "	<cmdframe x=\"" << frame.p.x() << "\" y=\"" << frame.p.y() << "\" z=\"" << frame.p.z() << "\" a=\""
				<< y << "\" b=\"" << p << "\" c=\"" << r << "\" />\r\n";

		ret << "</controller>\r\n";
		return ret.str();
	}

	string ybBaseHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;
		DeviceInstanceT<youBotBase> di(path[1]);

		if (di.getDevice() == 0)
			return "";

		double a, b, c;
		KDL::Frame pos = di->getMeasuredBasePosition();
		pos.M.GetRPY(c, b, a);

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/kuka/youbot/base.xsl\" ?>\r\n"
				<< "<controller name=\"" << path[1] << "\" x=\"" << pos.p.x() << "\" y=\"" << pos.p.y() << "\" yaw=\"" << a
				<< "\">\r\n";
		ret << "</controller>\r\n";
		return ret.str();
	}

} /* namespace kuka_youbot */
