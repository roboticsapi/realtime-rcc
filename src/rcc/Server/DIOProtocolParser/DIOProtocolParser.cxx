/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/Server/DIOProtocolParser/DIOProtocolParser.hpp>
#include "DIOProtocolP_inc.h"

namespace DIOProtocolP
{

	DIOProtocolParser::DIOProtocolParser()
	{
		// TODO Auto-generated constructor stub

	}

	DIOProtocolParser::~DIOProtocolParser()
	{
		// TODO Auto-generated destructor stub
	}

	std::unique_ptr<DIOCommand> DIOProtocolParser::parse(const std::string& command)
	{
		Scanner scanner((unsigned char*) command.c_str(), command.length());
		Parser parser(&scanner);

		parser.Parse();

		if (parser.errors->count > 0) {
			// errors = parser.errors->getMessage();
			return 0;
		}

		return std::move(parser.diocommand);
	}

} /* namespace DIOProtocolP */
