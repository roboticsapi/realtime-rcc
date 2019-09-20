/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "IOWeb.hpp"
#include "../interface/IOInterface.hpp"
#include <rcc/DeviceInstanceT.hpp>

namespace IO
{
	using namespace std;
	using namespace RPI;

	IOWeb::IOWeb()
	{
	}

	IOWeb::~IOWeb()
	{
	}

	string IOWeb::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;
		DeviceInstanceT<IOInterface> di(path[1]);

		if (method == "POST")
		{
			int id = getVarT<int>(data, "id", -1);
			if (id >= 0)
			{
				if (getVar(data, "type") == "digout")
				{
					bool value = !getVarT<bool>(data, "value", false);
					di->setDigitalOut(id, value);
				} else if (getVar(data, "type") == "anout")
				{
					double value = getVarT<double>(data, "value", 0);
					di->setAnalogOut(id, value);
				}
			}

			ret << "HTTP/1.1 303 See Other\r\n" << "Location: io\r\n" << "\r\n";
			return ret.str();
		}

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/io/io.xsl\" ?>\r\n";

		ret << "<iointerface name=\"" << path[1] << "\" numdigin=\"" << di->getNumDigitalIn() << "\" numdigout=\""
				<< di->getNumDigitalOut() << "\" numanin=\"" << di->getNumAnalogIn() << "\" numanout=\""
				<< di->getNumAnalogOut() << "\">\r\n";

		// IO Values
		ret << "\t<iodigin>\r\n";
		for (unsigned int i = 0; i < di->getNumDigitalIn(); i++)
		{
			ret << "\t\t<io id=\"" << i << "\" value=\"" << di->getDigitalIn(i) << "\"/>\n";
		}

		ret << "\t</iodigin>\r\n";
		ret << "\t<iodigout>\r\n";

		for (unsigned int i = 0; i < di->getNumDigitalOut(); i++)
		{
			ret << "\t\t<io id=\"" << i << "\" value=\"" << di->getDigitalOut(i) << "\"/>\n";
		}

		ret << "\t</iodigout>\r\n";
		ret << "\t<ioanin>\r\n";

		for (unsigned int i = 0; i < di->getNumAnalogIn(); i++)
		{
			ret << "\t\t<io id=\"" << i << "\" value=\"" << di->getAnalogIn(i) << "\"/>\n";
		}

		ret << "\t</ioanin>\r\n";
		ret << "\t<ioanout>\r\n";

		for (unsigned int i = 0; i < di->getNumAnalogOut(); i++)
		{
			ret << "\t\t<io id=\"" << i << "\" value=\"" << di->getAnalogOut(i) << "\"/>\n";
		}

		ret << "\t</ioanout>\r\n";

		ret << "</iointerface>\r\n";

		return ret.str();
	}

}/* namespace IO */
