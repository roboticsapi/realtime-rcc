/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "NetAST.hpp"
#include <sstream>

namespace RPI
{

	using namespace std;

	rpi_port::rpi_port() :
			name(), from_module(), from_port(), debug(0)
	{

	}

	string rpi_port::toXML(const string& tagname) const
	{
		stringstream ret;

		ret << "<" << tagname << " name=\"" << name << "\" frommodule=\"" << from_module << "\" fromport=\"" << from_port << "\"";
		if(debug > 0)
			ret << " debug=\"" << debug << "\"";
		ret << "/>\n";

		return ret.str();
	}


	rpi_parameter::rpi_parameter() :
			name(), value()
	{

	}

	string rpi_parameter::toXML() const
	{
		stringstream ret;

		ret << "<parameter name=\"" << name << "\" value=\"" << value << "\"/>\n";

		return ret.str();
	}

	rpi_module::rpi_module() :
			id(), type(), parameters(), inPorts()
	{

	}

	string rpi_module::toXML() const
	{
		stringstream ret;

		ret << "<module id=\"" << id << "\" type=\"" << type << "\">\n";

		for(list<rpi_parameter>::const_iterator it = parameters.begin(); it != parameters.end(); ++it)
			ret << it->toXML();
		for(list<rpi_port>::const_iterator it = inPorts.begin(); it != inPorts.end(); ++it)
			ret << it->toXML("port");

		ret << "</module>\n";

		return ret.str();
	}

	rpi_fragment::rpi_fragment() :
			id(), fragments(), modules(), outPorts(), inPorts()
	{

	}

	std::string rpi_fragment::toDIO() const
	{
		stringstream ret;
		ret << "{";
		bool needComma = false;
		for(auto it = fragments.begin(); it != fragments.end(); ++it) {
			if(needComma) ret << ","; needComma = true;
			ret << it->id << "=" << it->toDIO();
			ret << "("; bool iComma = false;
			for(auto it2 = it->inPorts.begin(); it2 != it->inPorts.end(); ++it2) {
				if(iComma) ret << ","; iComma = true;
				ret << it2->name << "=";
				if(it2->from_module == id)
					ret << "parent." + it2->from_port;
				else
					ret << it2->from_module + "." + it2->from_port;
			}
			ret << ")";
		}
		for(auto it = modules.begin(); it != modules.end(); ++it) {
			if(needComma) ret << ","; needComma = true;
			ret << it->id << "=" << it->type;
			ret << "("; bool iComma = false;
			for(auto it2 = it->inPorts.begin(); it2 != it->inPorts.end(); ++it2) {
				if(iComma) ret << ","; iComma = true;
				ret << it2->name;
				if(it2->debug != 0) ret << "[" << it2->debug << "]";
				ret << "=";
				if(it2->from_module == id)
					ret << "parent." + it2->from_port;
				else
					ret << it2->from_module + "." + it2->from_port;
			}
			for(auto it2 = it->parameters.begin(); it2 != it->parameters.end(); ++it2) {
				if(iComma) ret << ","; iComma = true;
				ret << it2->name << "='" << it2->value << "'";
			}
			ret << ")";
		}
		for(auto it = outPorts.begin(); it != outPorts.end(); ++it) {
			if(needComma) ret << ","; needComma = true;
			ret << it->name << "=" << it->from_module << "." << it->from_port;
		}
		ret << "}";
		return ret.str();
	}

	string rpi_fragment::toXML() const
	{
		stringstream ret;

		if(id == "ROOT")
			ret << "<rpinet id=\"";
		else
			ret << "<fragment id=\"";

		ret << this->id << "\">\n";

		for(list<rpi_fragment>::const_iterator it = fragments.begin(); it != fragments.end(); ++it)
			ret << it->toXML();
		for(list<rpi_module>::const_iterator it = modules.begin(); it != modules.end(); ++it)
			ret << it->toXML();
		for(list<rpi_port>::const_iterator it = inPorts.begin(); it != inPorts.end(); ++it)
			ret << it->toXML("inport");
		for(list<rpi_port>::const_iterator it = outPorts.begin(); it != outPorts.end(); ++it)
			ret << it->toXML("outport");

		if(id == "ROOT")
			ret << "</rpinet>\n";
		else
			ret << "</fragment>\n";

		return ret.str();
	}

}
