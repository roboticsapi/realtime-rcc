/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef NETPARSER_HPP_
#define NETPARSER_HPP_

#include "NetAST.hpp"
#include "rapidxml/rapidxml.hpp"
#include <exception>
#include <stack>

namespace RPI
{
	class NetParserException: public std::exception
	{

	};

	class NetParser
	{
	public:
		NetParser();
		virtual ~NetParser();

		static rpi_fragment parseNet(const std::string& netxml);
	private:
		static void parseFragment(rapidxml::xml_node<>* fragmentnode, rpi_fragment& container);
		static rpi_port parsePortNode(rapidxml::xml_node<>* portnode);
	};

	class NetParserCStr
	{
	public:
		NetParserCStr(const std::string& str);
		virtual ~NetParserCStr();

		char* data();
	private:
		char* m_buffer;
	};

}
#endif /* NETPARSER_HPP_ */
