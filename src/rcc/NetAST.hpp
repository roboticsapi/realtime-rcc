/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef NETAST_HPP_
#define NETAST_HPP_

#include <string>
#include <list>

namespace RPI
{
	class rpi_port
	{
	public:
		rpi_port();
		std::string toXML(const std::string& tagname) const;
		std::string name;
		std::string from_module;
		std::string from_port;
		double debug;
	};

	class rpi_parameter
	{
	public:
		rpi_parameter();
		std::string toXML() const;
		std::string name;
		std::string value;
	};

	class rpi_fragment;

	class rpi_module
	{
	public:
		rpi_module();
		std::string toXML() const;
		std::string id;
		std::string type;
		std::list<rpi_parameter> parameters;
		std::list<rpi_port> inPorts;
	};

	class rpi_fragment
	{
	public:
		rpi_fragment();
		std::string toXML() const;
		std::string toDIO() const;
		std::string id;
		std::list<rpi_fragment> fragments;
		std::list<rpi_module> modules;
		std::list<rpi_port> outPorts;
		std::list<rpi_port> inPorts;
	};


}

#endif /* NETAST_HPP_ */
