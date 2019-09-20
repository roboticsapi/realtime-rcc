/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DIRECTIONETCONVERTER_HPP_
#define DIRECTIONETCONVERTER_HPP_

#include "../../NetAST.hpp"
#include "DirectIONet.h"

namespace RPI
{

	class DirectIONetConverter
	{
	public:
		rpi_fragment convertNet(const DirectIONet::AstFragment&);

		DirectIONetConverter();
		virtual ~DirectIONetConverter();

	private:
		void parseFragment(const DirectIONet::AstFragment&, rpi_fragment& infragment);
		void parseRealPrimitive(const DirectIONet::AstPrimitive&, rpi_fragment&, const std::string& name);
		rpi_port parsePortRef(const DirectIONet::AstPortRef&, rpi_fragment&, const std::string& modulename, const std::string& portname);
		std::string generateUniqueID();

		int counter;
	};

} /* namespace RPI */
#endif /* DIRECTIONETCONVERTER_HPP_ */
