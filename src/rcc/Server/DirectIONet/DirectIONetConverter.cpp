/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "DirectIONetConverter.hpp"

namespace RPI
{
	using namespace std;

	DirectIONetConverter::DirectIONetConverter() :
			counter(0)
	{

	}

	DirectIONetConverter::~DirectIONetConverter()
	{

	}

	rpi_fragment DirectIONetConverter::convertNet(const DirectIONet::AstFragment& astfragment)
	{
		rpi_fragment newfragment;

		newfragment.id = "ROOT";
		parseFragment(astfragment, newfragment);

		return newfragment;
	}

	void DirectIONetConverter::parseFragment(const DirectIONet::AstFragment& astfragment, rpi_fragment& infragment)
	{
		for (map<string, DirectIONet::AstPrimitivePtr>::const_iterator it = astfragment.primitives.begin();
				it != astfragment.primitives.end(); ++it)
			parseRealPrimitive(*it->second, infragment, it->first);

		for (map<string, DirectIONet::AstPortRefPtr>::const_iterator it = astfragment.outPorts.begin();
				it != astfragment.outPorts.end(); ++it)
		{
			rpi_port port = parsePortRef(*it->second, infragment, infragment.id, it->first);

			infragment.outPorts.push_back(port);
		}


	}

	void DirectIONetConverter::parseRealPrimitive(const DirectIONet::AstPrimitive& astprim, rpi_fragment& fragment,
			const string& name)
	{
		switch (astprim.primtype)
		{
		case DirectIONet::AstPrimitive::PrimReference:
			// This must not happen here, a real primitive or fragment is required
			break;
		case DirectIONet::AstPrimitive::PrimFragment:
		{
			rpi_fragment childfrag;
			childfrag.id = name;
			parseFragment(astprim.fragment, childfrag);

			for (map<string, DirectIONet::AstPortRefPtr>::const_iterator it = astprim.inPorts.begin();
					it != astprim.inPorts.end(); ++it)
			{
				rpi_port port = parsePortRef(*it->second, fragment, childfrag.id, it->first);

				childfrag.inPorts.push_back(port);
			}

			fragment.fragments.push_back(childfrag);
		}
			break;
		case DirectIONet::AstPrimitive::PrimDefinition:
		{
			rpi_module mod;
			mod.id = name;
			mod.type = astprim.type;

			for (map<string, string>::const_iterator it = astprim.parameters.begin(); it != astprim.parameters.end();
					++it)
			{
				rpi_parameter param;
				param.name = it->first;
				param.value = it->second;
				mod.parameters.push_back(param);
			}

			for (map<string, DirectIONet::AstPortRefPtr>::const_iterator it = astprim.inPorts.begin();
					it != astprim.inPorts.end(); ++it)
			{
				rpi_port port = parsePortRef(*it->second, fragment, mod.id, it->first);

				mod.inPorts.push_back(port);
			}

			fragment.modules.push_back(mod);

		}
			break;
		}
	}

	rpi_port DirectIONetConverter::parsePortRef(const DirectIONet::AstPortRef& portref, rpi_fragment& fragment, const string& modulename, const string& portname)
	{
		rpi_port port;
		port.debug = portref.debug;
		port.name = portname;
		port.from_port = portref.name;

		switch (portref.prim.primtype)
		{
		case DirectIONet::AstPrimitive::PrimReference:
		{
			// Named module
			if (portref.prim.name != "parent")
			{
				port.from_module = portref.prim.name;
			} else
			{
				port.from_module = fragment.id;
			}
		}
			break;
		case DirectIONet::AstPrimitive::PrimDefinition:
		{
			// Anonymous new module
			string anonname = modulename + "." + portname + ".source"; // generateUniqueID();
			parseRealPrimitive(portref.prim, fragment, anonname);
			port.from_module = anonname;
		}
			break;
		case DirectIONet::AstPrimitive::PrimFragment:
		{
			// Anonymous new fragment
			string anonname = modulename + "." + portname + ".source"; // generateUniqueID();
			rpi_fragment anonfrag;
			anonfrag.id = anonname;
			parseFragment(portref.prim.fragment, anonfrag);
			port.from_module = anonname;
		}
			break;
		}

		return port;
	}

	string DirectIONetConverter::generateUniqueID()
	{
		stringstream str;
		str << "..anonymous." << counter++;
		return str.str();
	}

} /* namespace RPI */
