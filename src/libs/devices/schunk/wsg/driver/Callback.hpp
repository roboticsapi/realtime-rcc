/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CALLBACK_HPP_
#define CALLBACK_HPP_

#include "../interface/StatusCode.hpp"

namespace schunkwsg
{
	typedef unsigned char  uint8;
	typedef unsigned short uint16;

	class Callback
	{
	public:

		Callback() {};
		virtual ~Callback() {};
		virtual void messageReceived(uint8 command_id, StatusCode status_code, const uint8* payload, uint16 size) = 0;
	};

} /* namespace schunkwsg */
#endif /* CALLBACK_HPP_ */
