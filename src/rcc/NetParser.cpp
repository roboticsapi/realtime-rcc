/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "NetParser.hpp"
#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rpixmlutil.hpp"

#include "boost/algorithm/string.hpp"

namespace RPI
{
	using namespace std;
	using namespace rapidxml;

	NetParser::NetParser()
	{
	}

	NetParser::~NetParser()
	{
	}

	rpi_fragment NetParser::parseNet(const string& netxml) throw (NetParserException)
	{
		rpi_fragment net;
		net.id = "ROOT";

		try
		{
			NetParserCStr cstr(netxml);
			xml_document<> doc;
			doc.parse<0>(cstr.data());

			parseFragment(doc.first_node(), net);
		} catch (...)
		{
			throw NetParserException();
		}

		return net;
	}

	void NetParser::parseFragment(xml_node<>* fragmentnode, rpi_fragment& container)
	{
		// read sub-fragments
		for(xml_node<>* subfragment = fragmentnode->first_node("fragment", 0, false); subfragment; subfragment = subfragment->next_sibling("fragment", 0, false))
		{
			rpi_fragment newfrag;
			newfrag.id = getAttribute(subfragment, "id");
			container.fragments.push_back(newfrag);
			parseFragment(subfragment, container.fragments.back());
		}

		// read modules
		for(xml_node<>* modulenode = fragmentnode->first_node("module", 0, false); modulenode; modulenode = modulenode->next_sibling("module", 0, false))
		{
			rpi_module newmod;
			newmod.id = getAttribute(modulenode, "id");
			newmod.type = getAttribute(modulenode, "type");

			// parameter for module
			for(xml_node<>* parameternode = modulenode->first_node("parameter", 0, false); parameternode; parameternode =parameternode->next_sibling("parameter", 0, false))
			{
				rpi_parameter newparam;
				newparam.name = getAttribute(parameternode, "name");
				newparam.value = getAttribute(parameternode, "value");

				newmod.parameters.push_back(newparam);
			}

			// ports for module
			for(xml_node<>* portnode = modulenode->first_node("port", 0, false); portnode; portnode=portnode->next_sibling("port", 0, false))
			{
				newmod.inPorts.push_back(parsePortNode(portnode));
			}

			container.modules.push_back(newmod);
		}

		// Parse inPorts
		for(xml_node<>* portnode = fragmentnode->first_node("inport", 0, false); portnode; portnode=portnode->next_sibling("inport", 0, false))
		{
			container.inPorts.push_back(parsePortNode(portnode));
		}

		// Parse outPorts
		for(xml_node<>* portnode = fragmentnode->first_node("outport", 0, false); portnode; portnode=portnode->next_sibling("outport", 0, false))
		{
			container.outPorts.push_back(parsePortNode(portnode));
		}
	}

	rpi_port NetParser::parsePortNode(xml_node<>* portnode)
	{
		rpi_port newport;
		newport.name = getAttribute(portnode, "name");
		newport.from_module = getAttribute(portnode, "frommodule");
		newport.from_port = getAttribute(portnode, "fromport");
		newport.debug = atof(getAttribute(portnode, "debug", "0").c_str());

		return newport;

	}

	NetParserCStr::NetParserCStr(const string& str)
	{
		m_buffer = new char[str.length() + 1];
		std::strcpy(m_buffer, str.c_str());
	}

	NetParserCStr::~NetParserCStr()
	{
		delete[] m_buffer;
	}

	char* NetParserCStr::data()
	{
		return m_buffer;
	}

}
