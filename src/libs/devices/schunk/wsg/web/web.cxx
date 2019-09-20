/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "../simulation/SchunkWsgSimDevice.hpp"
#include "web.hpp"
#include "../interface/SchunkWsgDevice.hpp"
#include <rcc/DeviceInstanceT.hpp>

namespace schunkwsg
{
	using namespace std;

	SchunkWsgDeviceHandler::SchunkWsgDeviceHandler()
	{
	}

	SchunkWsgDeviceHandler::~SchunkWsgDeviceHandler()
	{
	}

	string SchunkWsgDeviceHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;
		RPI::DeviceInstanceT<SchunkWsgDevice> di(path[1]);
		SchunkWsgDevice* device = di.getDevice();

		if (device == 0)
			return "";

		if (method=="POST") {
			RPI::DeviceInstanceT<SchunkWsgSimDevice> di(path[1]);
			SchunkWsgSimDevice* simDevice = di.getDevice();
			if (simDevice!=0 && path[1]==getVar(data, "name") && getVar(data, "partLost")=="true") {
				simDevice->setPartLost();
			}
		}

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/schunkwsg/device.xsl\" ?>\r\n"
				<< "<controller name=\"" << path[1]
				<< "\" busy=\"" << (device->isBusy() ? "true" : "false")
				<< "\" statusCode=\"" << StatusCodeNames[device->getStatusCode()]
				<< "\" openingWidth=\"" << device->getOpeningWidth()
				<< "\" velocity=\"" << device->getVelocity()
				<< "\" force=\"" << device->getForce()
				<< "\" graspingState=\"" << GraspingStateNames[device->getGraspingState()]
				<< "\">\r\n";
		ret << "</controller>\r\n";
		return ret.str();
	}

} /* namespace schunkwsg */
