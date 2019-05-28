/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DIOPROTOCOLPARSER_HPP_
#define DIOPROTOCOLPARSER_HPP_

#include "DIOProtocolP.hpp"

namespace DIOProtocolP
{

	class DIOProtocolParser
	{
	public:
		DIOProtocolParser();
		virtual ~DIOProtocolParser();

		static std::unique_ptr<DIOCommand> parse(const std::string& command);
	};

} /* namespace DIOProtocolP */

#endif /* DIOPROTOCOLPARSER_HPP_ */
