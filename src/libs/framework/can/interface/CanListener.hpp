/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANLISTENER_HPP_
#define CANLISTENER_HPP_

#include "CanMessage.hpp"

namespace can
{

	class CanListener
	{
	public:
		CanListener() {};
		virtual ~CanListener() {};
		virtual void dataReceived(const CanMessage* message) = 0;
	};

} /* namespace can */
#endif /* CANLISTENER_HPP_ */
