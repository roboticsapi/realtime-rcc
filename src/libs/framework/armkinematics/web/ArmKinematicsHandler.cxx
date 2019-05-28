/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "ArmKinematicsHandler.hpp"
#include "../interface/ArmKinematicsInterface.hpp"
#include <rcc/DeviceFactory.hpp>
#include <rcc/DeviceInstanceT.hpp>

#include <string>

namespace armkinematics
{
	using namespace std;
	using namespace RPI;

	string ArmKinematicsHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;
		DeviceInstanceT<ArmKinematicsInterface> di(path[1]);
		if (di.getDevice() == 0)
			return "";

		int jcnt = di.getDevice()->getJointCount();

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/armkinematics/kinematics.xsl\" ?>\r\n"
				<< "<kinematics>\r\n";

		Array<double> joints, hintjoints;
		joints.resize(jcnt);
		hintjoints.resize(jcnt);

		for (int i = 1; i <= jcnt; ++i)
		{
			stringstream jname;
			jname << "j" << i;
			string val = getVar(get, jname.str());
			if (val != "")
			{
				joints[i - 1] = atof(val.c_str());
			} else
			{
				joints[i - 1] = 0;
			}
		}

		if (getVar(get, "x") != "" || getVar(get, "y") != "" || getVar(get, "z") != "" || getVar(get, "a") != ""
				|| getVar(get, "b") != "" || getVar(get, "c") != "")
		{
			KDL::Frame pos;
			pos = di.getDevice()->Kin(joints);

			double x = pos.p.x(), y = pos.p.y(), z = pos.p.z();
			double a, b, c;
			pos.M.GetRPY(c, b, a);

			if (getVar(get, "x") != "")
				x = atof(getVar(get, "x").c_str());
			if (getVar(get, "y") != "")
				y = atof(getVar(get, "y").c_str());
			if (getVar(get, "z") != "")
				z = atof(getVar(get, "z").c_str());
			if (getVar(get, "a") != "")
				a = atof(getVar(get, "a").c_str());
			if (getVar(get, "b") != "")
				b = atof(getVar(get, "b").c_str());
			if (getVar(get, "c") != "")
				c = atof(getVar(get, "c").c_str());

			pos = KDL::Frame(KDL::Rotation::RPY(c, b, a), KDL::Vector(x, y, z));

			joints.copyTo(hintjoints);

			di.getDevice()->InvKin(hintjoints, pos, joints);
		}

		ret << std::fixed;
		for (int i = 1; i <= jcnt; ++i)
		{
			ret << "<joint nr=\"" << i << "\" value=\"" << joints[i - 1] << "\"/>\r\n";
		}

		KDL::Frame frame;
		frame = di.getDevice()->Kin(joints);

		double r, p, y;
		frame.M.GetRPY(r, p, y);
		ret << "<frame x=\"" << frame.p.x() << "\" y=\"" << frame.p.y() << "\" z=\"" << frame.p.z() << "\" a=\"" << y
				<< "\" b=\"" << p << "\" c=\"" << r << "\" />\r\n";

		ret << "</kinematics>\r\n";
		return ret.str();
	}

} /* namespace ethercat */
