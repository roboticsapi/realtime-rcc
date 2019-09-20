/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "web.hpp"
#include "../driver/SchunkLwaDriver.hpp"
#include <rcc/DeviceInstanceT.hpp>

namespace schunk_lwa {

	using namespace std;
	using namespace RPI;

	string SchunkLWAHandler::handleRequest(vector<string> path, string method, string data, string get) {

		stringstream ret;
		DeviceInstanceT<SchunkLwaDriver> di(path[1]);

		if (method == "GET") {
			if(getVarT<int>(get, "ip", -1) == 1) {
				di->startInterpolatedPositionMode();
			} else if (getVarT<int>(get, "home", -1) == 1) {
				int id = getVarT<int>(get, "joint", -1);
				if (id >= 0) {
					di->homeJoint(id);
				} else {
					di->homeJoints();
				}
			}
		}

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><!--?xml-stylesheet type=\"text/xsl\" href=\"/xsl/schunklwa/device.xsl\" ?-->\r\n";

		ret << "<device name=\"" << path[1] << "\">\r\n";
			for (int i=0; i<6; i++) {
				ret << "Joint " << i << " status: " << di->current_hook_states[i] << std::endl;
			}
		ret << "</device>\r\n";

		return ret.str();
	}

}/* namespace IO */
